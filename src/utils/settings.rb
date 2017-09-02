#!/usr/bin/env ruby

# This file is part of INAV.
#
# author: Alberto Garcia Hierro <alberto@garciahierro.com>
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this file,
# You can obtain one at http://mozilla.org/MPL/2.0/.
#
# Alternatively, the contents of this file may be used under the terms
# of the GNU General Public License Version 3, as described below:
#
# This file is free software: you may copy, redistribute and/or modify
# it under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see http://www.gnu.org/licenses/.

require 'fileutils'
require 'open3'
require 'rbconfig'
require 'set'
require 'shellwords'
require 'stringio'
require 'tmpdir'
require 'yaml'

DEBUG = false
INFO = false

def dputs(s)
    puts s if DEBUG
end

def lputs(s)
    puts if DEBUG or INFO
end

class Object
    def is_number_kind?
        self.kind_of?(Integer) || self.kind_of?(Float)
    end
end

class String
    def is_number?
      true if Float(self) rescue false
    end
end

class StringIO
    def write_byte(b)
        self << [b].pack("C*")
    end

    def write_uvarint(x)
        while x >= 0x80
            write_byte((x & 0xFF) | 0x80)
            x >>= 7
        end
        write_byte(x)
    end

    def to_carr
        return string.bytes.to_s.sub('[', '{').sub(']', '}')
    end
end

class NameEncoder
    attr_reader :max_length

    def initialize(names, max_length)
        @names = names
        @max_length = max_length
        # Key is word, value is number of uses
        @words = Hash.new(0)
        # Most used words first
        @words_by_usage = []
        # Words that shouldn't be split because
        # their encoding is too long
        @non_split = Set.new
        # Key is the name, value is its encoding
        @encoded = Hash.new

        update_words
        encode_names
    end

    def uses_byte_indexing
        @words.length < 255
    end

    def words
        @words_by_usage
    end

    def estimated_size(settings_count)
        size = 0
        @words.each do |word, count|
            size += word.length + 1
        end
        return size + @max_length * settings_count
    end

    def format_encoded_name(name)
        encoded = @encoded[name]
        raise "Name #{name} was not encoded" if encoded == nil
        return encoded.to_carr
    end

    private
    def split_words(name)
        if @non_split.include?(name)
            return [name]
        end
        return name.split('_')
    end

    def update_words
        @words.clear
        @names.each do |name|
            split_words(name).each do |word|
                @words[word] += 1
            end
        end
        # Sort by usage, then alphabetically
        @words_by_usage = @words.keys().sort do |x, y|
            ux = @words[x]
            uy = @words[y]
            if ux != uy
                uy <=> ux
            else
                x <=> y
            end
        end
    end

    def encode_names
        @encoded.clear()
        @names.each do |name|
            buf = StringIO.new
            split_words(name).each do |word|
                pos = @words_by_usage.find_index(word)
                raise "Word #{word} not found in words array" if pos == nil
                # Zero indicates end of words, first word in
                # the array starts at 1 ([0] is NULL).
                p = pos + 1
                if uses_byte_indexing
                    buf.write_byte(p)
                else
                    buf.write_uvarint(p)
                end
            end
            if buf.length > @max_length
                # TODO: print in verbose mode
                # fmt.Printf("encoding %q took %d bytes (>%d), adding it as single word\n", v, len(data), e.MaxEncodedLength)
                @non_split << name
                update_words
                return encode_names
            end
            while buf.length < @max_length
                buf.write_byte(0)
            end
            @encoded[name] = buf
        end
    end
end

class ValueEncoder
    attr_reader :values

    def initialize(values, constants)
        min = 0
        max = 0

        valuesHash = Hash.new(0)
        values.each do |v|
            min = [min, v].min
            max = [max, v].max
            valuesHash[v] += 1
        end

        # Sorted by usage, most used first
        @values = valuesHash.keys().sort do |x, y|
            ux = valuesHash[x]
            uy = valuesHash[y]
            if ux != uy
                uy <=> ux
            else
                x <=> y
            end
        end

        @constants = constants

        @min = min
        @max = max
    end

    def min_type
        [8, 16, 32].each do |x|
            if @min >= -(2**(x-1))
                return "int#{x}_t"
            end
        end
        raise "cannot represent minimum value #{@min} with int32_t"
    end

    def max_type
        [8, 16, 32].each do |x|
            if @max < 2**x
                return "uint#{x}_t"
            end
        end
        raise "cannot represent maximum value #{@max} with uint32_t"
    end

    def index_bytes
        bits = Math.log2(@values.length).ceil
        bytes = (bits / 8.0).ceil.to_i
        if bytes > 1
            raise "too many bytes required for value index: #{bytes}"
        end
        return bytes
    end

    def encode_values(min, max)
        buf = StringIO.new
        encode_value(buf, min)
        encode_value(buf, max)
        return buf.to_carr
    end

    private
    def encode_value(buf, val)
        v = val || 0
        if !v.is_number_kind?
            v = @constants[val]
            if v == nil
                raise "Could not resolve constant #{val}"
            end
        end
        pos = @values.find_index(v)
        if pos < 0
            raise "Could not encode value not in array #{v}"
        end
        buf.write_byte(pos)
    end
end

OFF_ON_TABLE = Hash["name" => "off_on", "values" => ["OFF", "ON"]]

class Generator
    def initialize(src_root, settings_file)
        @src_root = src_root
        @settings_file = settings_file
        @output_dir = File.dirname(settings_file)

        @gpluspluspath = nil

        @count = 0
        @max_name_length = 0
        @tables = Hash.new
        @used_tables = Set.new
        @enabled_tables = Set.new

        @data = YAML.load_file(settings_file)

        initialize_tables
        check_conditions
        sanitize_fields
        initialize_name_encoder
        initialize_value_encoder
    end

    def write_files
        header_file = File.join(@output_dir, "settings_generated.h")
        impl_file = File.join(@output_dir, "settings_generated.c")

        write_header_file(header_file)
        write_impl_file(impl_file)
    end

    def print_stats
        puts "#{@count} settings"
        puts "words table has #{@name_encoder.words.length} words"
        word_idx = @name_encoder.uses_byte_indexing ? "byte" : "uvarint"
        puts "name encoder uses #{word_idx} word indexing"
        puts "each setting name uses #{@name_encoder.max_length} bytes"
        puts "#{@name_encoder.estimated_size(@count)} bytes estimated for setting name storage"
        values_size = @value_encoder.values.length * 4
        puts "value storage uses #{values_size} bytes"
        value_idx_size = @value_encoder.index_bytes * 2
        value_idx_total = value_idx_size * @count
        puts "value indexing uses #{value_idx_size} per setting, #{value_idx_total} bytes total"
        puts "#{value_idx_size+value_idx_total} bytes estimated for value storage"

        buf = StringIO.new
        buf << "#include \"fc/settings.h\"\n"
        buf << "char (*dummy)[sizeof(clivalue_t)] = 1;\n"
        stderr = compile_test_file(buf)
        puts "sizeof(clivalue_t) = #{/char \(\*\)\[(\d+)\]/.match(stderr)[1]}"
    end

    private

    def write_header_file(file)
        buf = StringIO.new
        buf << "#pragma once\n"
        # Write clivalue_t size constants
        buf << "#define CLIVALUE_MAX_NAME_LENGTH #{@max_name_length+1}\n" # +1 for the terminating '\0'
        buf << "#define CLIVALUE_ENCODED_NAME_MAX_BYTES #{@name_encoder.max_length}\n"
        if @name_encoder.uses_byte_indexing
            buf << "#define CLIVALUE_ENCODED_NAME_USES_BYTE_INDEXING\n"
        end
        buf << "#define CLIVALUE_TABLE_COUNT #{@count}\n"
        offset_type = "uint16_t"
        if can_use_byte_offsetof
            offset_type = "uint8_t"
        end
        buf << "typedef #{offset_type} clivalue_offset_t;\n"
        pgn_count = 0
        foreach_enabled_group do |group|
            pgn_count += 1
        end
        buf << "#define CLIVALUE_PGN_COUNT #{pgn_count}\n"
        # Write type definitions and constants for min/max values
        buf << "typedef #{@value_encoder.min_type} clivalue_min_t;\n"
        buf << "typedef #{@value_encoder.max_type} clivalue_max_t;\n"
        buf << "#define CLIVALUE_MIN_MAX_INDEX_BYTES #{@value_encoder.index_bytes*2}\n"
        # Write lookup table constants
        table_names = ordered_table_names()
        buf << "enum {\n"
        table_names.each do |name|
            buf << "\t#{table_constant_name(name)},\n"
        end
        buf << "\tLOOKUP_TABLE_COUNT,\n"
        buf << "};\n"
        # Write table pointers
        table_names.each do |name|
            buf << "extern const char *#{table_variable_name(name)}[];\n"
        end

        File.open(file, 'w') {|file| file.write(buf.string)}
    end

    def write_impl_file(file)
        buf = StringIO.new
        add_header = ->(h) {
            buf << "#include \"#{h}\"\n"
        }
        add_header.call("platform.h")
        add_header.call("config/parameter_group_ids.h")
        add_header.call("settings.h")

        foreach_enabled_group do |group|
            (group["headers"] || []).each do |h|
                add_header.call(h)
            end
        end

        # Write PGN arrays
        pgn_steps = []
        pgns = []
        foreach_enabled_group do |group|
            count = 0
            group["members"].each do |member|
                if is_condition_enabled(member["condition"])
                    count += 1
                end
            end
            pgn_steps << count
            pgns << group["name"]
        end

        buf << "const pgn_t cliValuePgn[] = {\n"
        pgns.each do |p|
            buf << "\t#{p},\n"
        end
        buf << "};\n"
        buf << "const uint8_t cliValuePgnCounts[] = {\n"
        pgn_steps.each do |s|
            buf << "\t#{s},\n"
        end
        buf << "};\n"

        # Write word list
        buf << "static const char *cliValueWords[] = {\n"
        buf << "\tNULL,\n"
        @name_encoder.words.each do |w|
            buf << "\t#{w.inspect},\n"
        end
        buf << "};\n"

        # Write the tables
        table_names = ordered_table_names()
        table_names.each do |name|
            buf << "const char *#{table_variable_name(name)}[] = {\n"
            tbl = @tables[name]
            tbl["values"].each do |v|
                buf << "\t#{v.inspect},\n"
            end
            buf << "};\n"
        end

        buf << "const lookupTableEntry_t cliLookupTables[] = {\n"
        table_names.each do |name|
            vn = table_variable_name(name)
            buf << "\t{ #{vn}, sizeof(#{vn}) / sizeof(char*) },\n"
        end
        buf << "};\n"

        # Write min/max values table
        buf << "const uint32_t cliValueMinMaxTable[] = {\n"
        @value_encoder.values.each do |v|
            buf <<  "\t#{v},\n"
        end
        buf << "};\n"

        case @value_encoder.index_bytes
        when 1
            buf << "typedef uint8_t clivalue_min_max_idx_t;\n"
            buf << "#define CLIVALUE_INDEXES_GET_MIN(val) (val->config.minmax.indexes[0])\n"
            buf << "#define CLIVALUE_INDEXES_GET_MAX(val) (val->config.minmax.indexes[1])\n"
        else
            raise "can't encode indexed values requiring #{@value_encoder.index_bytes} bytes"
        end

        # Write clivalues
        buf << "const clivalue_t cliValueTable[] = {\n"

        last_group = nil
        foreach_enabled_member do |group, member|
            if group != last_group
                last_group = group
                buf << "\t// #{group["name"]}\n"
            end

            buf << "\t{ #{@name_encoder.format_encoded_name(member["name"])}, "
            buf << "#{var_type(member["type"])} | #{value_type(group)}"
            tbl = member["table"]
            if tbl
                buf << " | MODE_LOOKUP"
                buf << ", .config.lookup = { #{table_constant_name(tbl)} }"
            else
                enc = @value_encoder.encode_values(member["min"], member["max"])
                buf <<  ", .config.minmax.indexes = #{enc}"
            end
            buf << ", offsetof(#{group["type"]}, #{member["field"]}) },\n"
        end
        buf << "};\n"

        File.open(file, 'w') {|file| file.write(buf.string)}
    end

    def var_type(typ)
        case typ
        when "uint8_t", "bool"
            return "VAR_UINT8"
        when "int8_t"
            return "VAR_INT8"
        when "uint16_t"
            return "VAR_UINT16"
        when "int16_t"
            return "VAR_INT16"
        when "uint32_t"
            return "VAR_UINT32"
        when "float"
            return "VAR_FLOAT"
        else
            raise "unknown variable type #{typ.inspect}"
        end
    end

    def value_type(group)
        return group["value_type"] || "MASTER_VALUE"
    end

    def is_condition_enabled(cond)
        return !cond || @true_conditions.include?(cond)
    end

    def foreach_enabled_member
        @data["groups"].each do |group|
            if is_condition_enabled(group["condition"])
                group["members"].each do |member|
                    if is_condition_enabled(member["condition"])
                        yield group, member
                    end
                end
            end
        end
    end

    def foreach_enabled_group
        last = nil
        foreach_enabled_member do |group, member|
            if last != group
                last = group
                yield group
            end
        end
    end

    def foreach_member
        @data["groups"].each do |group|
            group["members"].each do |member|
                yield group, member
            end
        end
    end

    def foreach_group
        last = nil
        foreach_member do |group, member|
            if last != group
                last = group
                yield group
            end
        end
    end

    def initialize_tables
        @data["tables"].each do |tbl|
            name = tbl["name"]
            if @tables.key?(name)
                raise "Duplicate table name #{name}"
            end
            @tables[name] = tbl
        end
    end

    def ordered_table_names
        @enabled_tables.to_a().sort()
    end

    def table_constant_name(name)
        upper = name.upcase()
        "TABLE_#{upper}"
    end

    def table_variable_name(name)
        "table_#{name}"
    end

    def can_use_byte_offsetof
        buf = StringIO.new
        foreach_enabled_member do |group, member|
            typ = group["type"]
            field = member["field"]
            buf << "static_assert(offsetof(#{typ}, #{field}) < 255, \"#{typ}.#{field} is too big\");\n"
        end
        stderr = compile_test_file(buf)
        return !stderr.include?("static assertion failed")
    end

    def mktmpdir
        # Use a temporary dir reachable by relative path
        # since g++ in cygwin fails to open files
        # with absolute paths
        tmp = File.join("obj", "tmp")
        FileUtils.mkdir_p(tmp) unless File.directory?(tmp)
        value = yield(tmp)
        FileUtils.remove_dir(tmp)
        value
    end

    def find_gplusplus
        # Look for the compiler in PATH manually, since there
        # are some issues with the built-in search by spawn()
        # on Windows if PATH contains spaces.
        return if @gpluspluspath
        dirs = (ENV["PATH"] || "").split(File::PATH_SEPARATOR)
        bin = "arm-none-eabi-g++"
        dirs.each do |dir|
            p = File.join(dir, bin)
            ['', '.exe'].each do |suffix|
                f = p + suffix
                if File.executable?(f)
                    dputs "Found #{bin} at #{f}"
                    @gpluspluspath = f
                    return
                end
            end
        end

        raise "Could not find #{bin} in PATH, looked in #{dirs}"
    end

    def join_args(args)
        if RbConfig::CONFIG['host_os'] =~ /mswin|mingw/
            return args.join(' ')
        end
        return Shellwords.join(args)
    end

    def compile_test_file(prog)
        find_gplusplus
        buf = StringIO.new
        # cstddef for offsetof()
        headers = ["target.h", "platform.h", "cstddef"]
        @data["groups"].each do |group|
            gh = group["headers"]
            if gh
                headers.concat gh
            end
        end
        headers.each do |h|
            if h
                buf << "#include \"#{h}\"\n"
            end
        end
        buf << "\n"
        buf << prog.string
        mktmpdir do |dir|
            file = File.join(dir, "test.cpp")
            File.open(file, 'w') {|file| file.write(buf.string)}
            cflags = Shellwords.split(ENV["CFLAGS"] || "")
            args = [@gpluspluspath]
            cflags.each do |flag|
                # Don't generate temporary files
                if flag == "" || flag == "-MMD" || flag == "-MP" || flag.start_with?("-save-temps")
                    next
                end
                if flag.start_with? "-std="
                    flag = "-std=c++11"
                end
                if flag.start_with? "-D'"
                    # Cleanup flag. Done by the shell when called from
                    # it but we must do it ourselves becase we're not
                    # calling the compiler via shell.
                    flag = "-D" + flag[3..-2]
                end
                args << flag
            end

            args << "-o" << File.join(dir, "test") << file
            dputs "Compiling #{buf.string}"
            stdout, stderr = Open3.capture3(join_args(args))
            dputs "Output: #{stderr}"
            stderr
        end
    end

    def check_conditions
        buf = StringIO.new
        conditions = Set.new
        add_condition = -> (c) {
            if c && !conditions.include?(c)
                conditions.add(c)
                buf << "#ifdef #{c}\n"
                buf << "#pragma message(#{c.inspect})\n"
                buf << "#endif\n"
            end
        }

        foreach_member do |group, member|
            add_condition.call(group["condition"])
            add_condition.call(member["condition"])
        end

        stderr = compile_test_file(buf)
        @true_conditions = Set.new
        stderr.scan(/#pragma message\(\"(.*)\"\)/).each do |m|
            @true_conditions << m[0]
        end
    end

    def sanitize_fields
        pending_types = Hash.new
        has_booleans = false

        foreach_enabled_member do |group, member|
            if !group["name"]
                raise "Missing group name"
            end
            if !member["name"]
                raise "Missing member name in group #{group["name"]}"
            end
            table = member["table"]
            if table
                if !@tables[table]
                    raise "Member #{member["name"]} references non-existing table #{table}"
                end
                @used_tables << table
            end
            if !member["field"]
                member["field"] = member["name"]
            end
            typ = member["type"]
            if !typ
                pending_types[member] = group
            elsif typ == "bool"
                has_booleans = true
                member["type"] = "uint8_t"
                member["table"] = OFF_ON_TABLE["name"]
            end
        end

        if has_booleans
            @tables[OFF_ON_TABLE["name"]] = OFF_ON_TABLE
            @used_tables << OFF_ON_TABLE["name"]
        end

        resolve_types pending_types unless !pending_types
        foreach_enabled_member do |group, member|
            @count += 1
            @max_name_length = [@max_name_length, member["name"].length].max
            if member["table"]
                @enabled_tables << member["table"]
            end
        end
    end

    def resolve_types(pending)
        # TODO: Loop to avoid reaching the maximum number
        # of errors printed by the compiler.
        prog = StringIO.new
        prog << "int main() {\n"
        ii = 0
        members = Hash.new
        pending.each do |member, group|
            var = "var_#{ii}"
            members[ii] = member
            ii += 1
            gt = group["type"]
            mf = member["field"]
            prog << "#{gt} #{var}; #{var}.#{mf}.__type_detect_;\n"
        end
        prog << "return 0;\n"
        prog << "};\n"
        stderr = compile_test_file(prog)
        stderr.scan(/var_(\d+).*?', which is of non-class type '(.*)'/).each do |m|
            member = members[m[0].to_i]
            case m[1]
            when "int8_t {aka signed char}"
                typ = "int8_t"
            when "uint8_t {aka unsigned char}"
                typ = "uint8_t"
            when "int16_t {aka short int}"
                typ = "int16_t"
            when "uint16_t {aka short unsigned int}"
                typ = "uint16_t"
            when "uint32_t {aka long unsigned int}"
                typ = "uint32_t"
            when "float"
                typ = "float"
            else
                raise "Unknown type #{m[1]} when resolving type for setting #{member["name"]}"
            end
            dputs "#{member["name"]} type is #{typ}"
            member["type"] = typ
        end
        # Make sure all types have been resolved
        foreach_enabled_member do |group, member|
            if !member["type"]
                raise "Could not resolve type for member #{member["name"]} in group #{group["name"]}"
            end
        end
    end

    def initialize_name_encoder
        names = []
        foreach_enabled_member do |group, member|
            names << member["name"]
        end
        best = nil
        (3..7).each do |v|
            enc = NameEncoder.new(names, v)
            if best == nil || best.estimated_size(@count) > enc.estimated_size(@count)
                best = enc
            end
        end
        dputs "Using name encoder with max_length = #{best.max_length}"
        @name_encoder = best
    end

    def initialize_value_encoder
        values = []
        constants = []
        add_value = -> (v) {
            v = v || 0
            if v.is_number_kind? || (v.class == String && v.is_number?)
                values << v.to_i
            else
                constants << v
            end
        }
        foreach_enabled_member do |group, member|
            add_value.call(member["min"])
            add_value.call(member["max"])
        end
        constantValues = resolve_constants(constants)
        # Count values used by constants
        constants.each do |c|
            values << constantValues[c]
        end
        @value_encoder = ValueEncoder.new(values, constantValues)
    end

    def resolve_constants(constants)
        return nil unless constants.length > 0
        s = Set.new
        constants.each do |c|
            s << c
        end
        dputs "#{constants.length} constants to resolve"
        # Since we're relying on errors rather than
		# warnings to find these constants, the compiler
		# might reach the maximum number of errors and stop
		# compilation, so we might need multiple passes.
        re = /required from 'class expr_(.*?)<(.*)>'/
        values = Hash.new
		while s.length > 0
            buf = StringIO.new
            buf << "template <int64_t V> class Fail {\n"
            # Include V in the static_assert so it's shown
            # in the error condition.
            buf << "static_assert(V == 42 && 0 == 1, \"FAIL\");\n"
            buf << "public:\n"
            buf << "Fail() {};\n"
            buf << "int64_t v = V\n"
            buf << "};\n"
            ii  = 0
            s.each do |c|
                cls = "expr_#{c}"
                buf << "template <int64_t V> class #{cls}: public Fail<V> {};\n"
                buf << "#{cls}<#{c}> var_#{ii};\n"
                ii += 1
            end
            stderr = compile_test_file(buf)
			matches = stderr.scan(re)
			if matches.length == 0
                puts stderr
                raise "No more matches looking for constants"
            end
            matches.each do |m|
                c = m[0]
				v = m[1]
				# gcc 6.3 includes an ul or ll prefix after the
				# constant expansion, while gcc 7.1 does not
                v = v.tr("ul", "")
                nv = v.to_i
                values[c] = nv
                s.delete(c)
                dputs "Constant #{c} resolved to #{nv}"
            end
        end
        values
    end
end

def usage
    puts "Usage: ruby #{__FILE__} <source_dir> <settings_file>"
end

if __FILE__ == $0

    verbose = ENV["V"] == "1"

    src_root = ARGV[0]
    settings_file = ARGV[1]
    if src_root.nil? || settings_file.nil?
        usage()
        exit(1)
    end

    gen = Generator.new(src_root, settings_file)
    gen.write_files()

    if verbose
        gen.print_stats()
    end

end
