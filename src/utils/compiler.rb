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

require 'digest'
require 'open3'
require 'rbconfig'
require 'shellwords'

class Compiler
    def initialize
        # Look for the compiler in PATH manually, since there
        # are some issues with the built-in search by spawn()
        # on Windows if PATH contains spaces.
        dirs = (ENV["PATH"] || "").split(File::PATH_SEPARATOR)
        bin = "arm-none-eabi-g++"
        dirs.each do |dir|
            p = File.join(dir, bin)
            ['', '.exe'].each do |suffix|
                f = p + suffix
                if File.executable?(f)
                    if @verbose
                        puts "Found #{bin} at #{f}"
                    end
                    @path = f
                    return
                end
            end
        end
        raise "Could not find #{bin} in PATH, looked in #{dirs}"
        @verbose = ENV["V"] == "1"
    end

    def default_args
        cflags = Shellwords.split(ENV["CFLAGS"] || "")
        args = [@path]
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
        return args
    end

    def run(input, output, args = nil)
        all_args = default_args
        if args
            all_args.push(*args)
        end
        all_args << "-o" << output << input
        stdout, stderr = Open3.capture3(join_args(all_args))
        return stdout, stderr
    end

    private

    def join_args(args)
        if RbConfig::CONFIG['host_os'] =~ /mswin|mingw/
            return args.join(' ')
        end
        return Shellwords.join(args)
    end
end
