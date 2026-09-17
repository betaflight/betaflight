/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Checks the `//!<` debug field annotations in src/main against the grammar
 * documented above DEBUG_SET() in src/main/build/debug.h.
 *
 * An annotation is the only record of what a debug field means, and it is read
 * outside this repository: the configurator's generate-debug-modes builds its
 * field tables from it, and fails rather than emit a field it cannot describe.
 * So a malformed annotation is not a comment typo - it stops that generator,
 * and it is found by whoever regenerates next, in another repository, long
 * after the pull request that wrote it. This test moves that failure to the
 * pull request.
 *
 * Two halves: the grammar is exercised against literal annotations, so the
 * checker itself is tested, and then run over the firmware sources.
 */

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <dirent.h>
#include <sys/stat.h>

extern "C" {
#include "platform.h"
#include "build/debug.h"
}

#include "gtest/gtest.h"

namespace {

#define ARRAY_LENGTH(x) (sizeof(x) / sizeof((x)[0]))

// The unit symbols a `[unit:...]` annotation may use, which is the list in
// src/main/build/debug.h. Every consumer needs a display rule for a symbol
// before firmware may write it, so a new unit is added here, in debug.h, in the
// Debug Field Annotations page on betaflight.com and in the configurator's
// src/js/debug_units.ts, in one change.
const char *const UNIT_SYMBOLS[] = {
    "s", "ms", "us",
    "Hz", "kHz", "MHz", "kbit/s",
    "rad", "rad/s",
    "deg", "dps", "dps2",
    "m", "cm", "cm2", "m/s", "cm/s", "cm/s2", "cm2/s2",
    "g", "g/s",
    "V", "A", "mAh",
    "degC", "Pa", "hPa",
    "rpm", "%", "dB", "dBm",
    "bytes", "ticks",
    // Device-native: the firmware stores the raw value, and only the flight
    // controller's own configuration can convert it.
    "gyroADC", "accADC", "accADC/s", "rcCommand", "eRPM",
};

const char ANNOTATION_MARKER[] = "//!<";
const char DEBUG_SET_NAME[] = "DEBUG_SET";

std::string trim(const std::string &s)
{
    size_t first = 0;
    while (first < s.size() && isspace((unsigned char)s[first])) {
        first++;
    }
    size_t last = s.size();
    while (last > first && isspace((unsigned char)s[last - 1])) {
        last--;
    }
    return s.substr(first, last - first);
}

bool startsWith(const std::string &s, const std::string &prefix)
{
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

bool endsWith(const std::string &s, const std::string &suffix)
{
    return s.size() >= suffix.size() && s.compare(s.size() - suffix.size(), suffix.size(), suffix) == 0;
}

bool isIdentifierChar(char c)
{
    return isalnum((unsigned char)c) || c == '_';
}

std::vector<std::string> split(const std::string &s, char separator)
{
    std::vector<std::string> parts;
    std::string part;
    std::istringstream stream(s);
    while (std::getline(stream, part, separator)) {
        parts.push_back(part);
    }
    if (!s.empty() && s[s.size() - 1] == separator) {
        parts.push_back("");
    }
    return parts;
}

bool isUnsignedInteger(const std::string &s)
{
    if (s.empty()) {
        return false;
    }
    for (size_t i = 0; i < s.size(); i++) {
        if (!isdigit((unsigned char)s[i])) {
            return false;
        }
    }
    return true;
}

bool isKnownUnitSymbol(const std::string &symbol)
{
    for (size_t i = 0; i < ARRAY_LENGTH(UNIT_SYMBOLS); i++) {
        if (symbol == UNIT_SYMBOLS[i]) {
            return true;
        }
    }
    return false;
}

// Whether the index argument is something tooling can evaluate without help.
// Betaflight writes a constant in capitals, so an index written that way is a
// #define or an enum member, as is a literal. A lower case identifier may still
// be a constant - only the generator, which reads the headers, can tell - so
// this stays with what is unambiguous from the call alone.
bool isCompileTimeIndex(const std::string &argument)
{
    if (isUnsignedInteger(argument)) {
        return true;
    }
    if (argument.empty() || !isupper((unsigned char)argument[0])) {
        return false;
    }
    for (size_t i = 0; i < argument.size(); i++) {
        const char c = argument[i];
        if (!isupper((unsigned char)c) && !isdigit((unsigned char)c) && c != '_') {
            return false;
        }
    }
    return true;
}

// `[index:2]`, `[index:0..2]` and `[index:0,2,4]`. Returns how many indices the
// spec names, or 0 when it is malformed.
int indicesNamedBy(const std::string &spec, std::vector<std::string> *errors)
{
    const std::string text = trim(spec);
    if (text.empty()) {
        errors->push_back("empty index spec: name the indices the call writes");
        return 0;
    }
    if (isUnsignedInteger(text)) {
        return 1;
    }

    const size_t range = text.find("..");
    if (range != std::string::npos) {
        const std::string first = trim(text.substr(0, range));
        const std::string last = trim(text.substr(range + 2));
        if (!isUnsignedInteger(first) || !isUnsignedInteger(last)) {
            errors->push_back("index range '" + text + "' is not a pair of indices");
            return 0;
        }
        if (atoi(last.c_str()) < atoi(first.c_str())) {
            errors->push_back("index range '" + text + "' ends before it starts");
            return 0;
        }
        return atoi(last.c_str()) - atoi(first.c_str()) + 1;
    }

    const std::vector<std::string> listed = split(text, ',');
    for (size_t i = 0; i < listed.size(); i++) {
        if (!isUnsignedInteger(trim(listed[i]))) {
            errors->push_back("index spec '" + text + "' is not an index, a '0..2' range or a '0,2,4' list");
            return 0;
        }
    }
    return (int)listed.size();
}

// `[unit:...]` is the unit of one LSB: an optional factor, which may be
// negative for a field holding the magnitude of a negative quantity, followed
// by an optional symbol. One of the two has to be there.
void checkUnit(const std::string &unit, std::vector<std::string> *errors)
{
    size_t i = 0;
    size_t digits = 0;
    bool signOrPoint = false;
    if (i < unit.size() && unit[i] == '-') {
        signOrPoint = true;
        i++;
    }
    while (i < unit.size() && isdigit((unsigned char)unit[i])) {
        digits++;
        i++;
    }
    if (i < unit.size() && unit[i] == '.') {
        signOrPoint = true;
        i++;
        while (i < unit.size() && isdigit((unsigned char)unit[i])) {
            digits++;
            i++;
        }
    }
    const bool hasFactor = digits > 0;
    if (!hasFactor && signOrPoint) {
        errors->push_back("unit '" + unit + "' has a sign or a decimal point with no factor");
        return;
    }

    const std::string symbol = unit.substr(i);
    if (symbol.empty()) {
        if (!hasFactor) {
            errors->push_back("empty unit: give a factor, a symbol, or omit the bracket for a plain integer");
        }
        return;
    }
    if (!isKnownUnitSymbol(symbol)) {
        errors->push_back("unknown unit symbol '" + symbol + "': add it to the list in build/debug.h, "
                          "to the Debug Field Annotations page and to the configurator's debug_units.ts, "
                          "or omit the bracket for a plain integer");
    }
}

void checkFlags(const std::string &flags, std::vector<std::string> *errors)
{
    if (trim(flags).empty()) {
        errors->push_back("empty flag list: name the bits lowest first, with '-' for a bit the field does not use");
        return;
    }
    const std::vector<std::string> names = split(flags, '|');
    for (size_t i = 0; i < names.size(); i++) {
        if (trim(names[i]).empty()) {
            errors->push_back("flag list has an empty name: use '-' for a bit the field does not use");
            return;
        }
    }
}

struct Annotation {
    std::string label;
    std::string enumType;   // empty unless the shape is an enum
    int indices;            // how many debug[n] the call writes
};

// The grammar, from build/debug.h:  //!< [index:<indices>] <label> [<shape>]
std::vector<std::string> checkAnnotation(const std::string &annotation, Annotation *parsed)
{
    std::vector<std::string> errors;
    std::string text = trim(annotation);

    parsed->label.clear();
    parsed->enumType.clear();
    parsed->indices = 1;

    if (text.empty()) {
        errors.push_back("empty annotation");
        return errors;
    }

    bool hasIndexSpec = false;
    if (text[0] == '[') {
        const size_t close = text.find(']');
        if (close == std::string::npos) {
            errors.push_back("unterminated '[' in '" + text + "'");
            return errors;
        }
        const std::string bracket = text.substr(1, close - 1);
        if (startsWith(bracket, "index:")) {
            hasIndexSpec = true;
            const int indices = indicesNamedBy(bracket.substr(strlen("index:")), &errors);
            parsed->indices = (indices > 0) ? indices : 1;
            text = trim(text.substr(close + 1));
        }
    }

    if (!text.empty() && text[text.size() - 1] == ']') {
        const size_t open = text.rfind('[');
        if (open == std::string::npos) {
            errors.push_back("']' with no matching '['");
            return errors;
        }
        const std::string shape = text.substr(open + 1, text.size() - open - 2);
        text = trim(text.substr(0, open));

        const size_t colon = shape.find(':');
        const std::string key = (colon == std::string::npos) ? "" : shape.substr(0, colon);
        const std::string value = (colon == std::string::npos) ? "" : shape.substr(colon + 1);
        if (key == "unit") {
            checkUnit(value, &errors);
        } else if (key == "enum") {
            if (!endsWith(value, "_e") || value.size() < 3) {
                errors.push_back("'" + value + "' is not an enum type name");
            } else {
                parsed->enumType = value;
            }
        } else if (key == "flags") {
            checkFlags(value, &errors);
        } else if (key == "index") {
            errors.push_back("the index spec goes in front of the label, not after it");
        } else {
            errors.push_back("bracket '[" + shape + "]' carries no key: "
                             "expected 'index:', 'unit:', 'enum:' or 'flags:'");
        }
    }

    const std::string label = trim(text);
    parsed->label = label;
    if (label.empty()) {
        errors.push_back("no label: say what the value is, in the words a pilot reads");
    }
    if (label.find('[') != std::string::npos || label.find(']') != std::string::npos) {
        errors.push_back("label '" + label + "' contains a bracket: use parentheses for a qualifier");
    }

    const size_t firstBrace = label.find('{');
    const size_t closeBrace = label.find('}');
    const size_t opened = std::count(label.begin(), label.end(), '{');
    const size_t closed = std::count(label.begin(), label.end(), '}');
    if (opened > 1) {
        errors.push_back("label '" + label + "' has more than one {a|b|c} group");
    } else if (opened != closed || (opened == 1 && closeBrace < firstBrace)) {
        errors.push_back("label '" + label + "' has an unmatched brace");
    } else if (opened == 1) {
        if (!hasIndexSpec) {
            errors.push_back("label '" + label + "' spells out one name per index, "
                             "but the call has no [index:...] spec");
        } else {
            const std::vector<std::string> names = split(label.substr(firstBrace + 1, closeBrace - firstBrace - 1), '|');
            bool named = true;
            for (size_t n = 0; n < names.size(); n++) {
                named = named && !trim(names[n]).empty();
            }
            if (!named) {
                errors.push_back("label '" + label + "' leaves one of its fields unnamed");
            } else if ((int)names.size() != parsed->indices) {
                std::ostringstream message;
                message << "label '" << label << "' names " << names.size()
                        << " field(s) for " << parsed->indices << " index(es)";
                errors.push_back(message.str());
            }
        }
    }

    return errors;
}

/*
 * Reading the sources.
 *
 * Only live code carries an annotation: a commented out DEBUG_SET() is not a
 * field the mode writes, and the grammar in debug.h is itself a block comment
 * full of examples.
 */

struct SourceLine {
    std::string code;       // the line with its comments removed and its string bodies blanked
    std::string annotation; // the text after //!<
    std::string include;    // the name in #include "...", which blanking would have eaten
    bool hasAnnotation;

    SourceLine() : hasAnnotation(false) {}
};

// `#include "sensors/battery.h"` from the raw line, since the scan below blanks
// what is between quotes.
std::string includedFile(const std::string &line)
{
    const std::string text = trim(line);
    if (text.empty() || text[0] != '#' || text.find("include") == std::string::npos) {
        return "";
    }
    const size_t open = text.find('"');
    if (open == std::string::npos) {
        return "";
    }
    const size_t close = text.find('"', open + 1);
    return (close == std::string::npos) ? "" : text.substr(open + 1, close - open - 1);
}

void readSource(const std::string &path, std::vector<SourceLine> *lines)
{
    std::ifstream file(path.c_str());
    std::string line;
    bool inBlockComment = false;

    while (std::getline(file, line)) {
        SourceLine source;
        if (!inBlockComment) {
            source.include = includedFile(line);
        }
        size_t i = 0;
        while (i < line.size()) {
            if (inBlockComment) {
                if (line[i] == '*' && i + 1 < line.size() && line[i + 1] == '/') {
                    inBlockComment = false;
                    i += 2;
                } else {
                    i++;
                }
                continue;
            }
            if (line[i] == '/' && i + 1 < line.size() && line[i + 1] == '*') {
                inBlockComment = true;
                i += 2;
                continue;
            }
            if (line[i] == '/' && i + 1 < line.size() && line[i + 1] == '/') {
                if (line.compare(i, strlen(ANNOTATION_MARKER), ANNOTATION_MARKER) == 0) {
                    source.hasAnnotation = true;
                    source.annotation = trim(line.substr(i + strlen(ANNOTATION_MARKER)));
                }
                break;
            }
            if (line[i] == '"' || line[i] == '\'') {
                const char quote = line[i];
                source.code += quote;
                i++;
                while (i < line.size() && line[i] != quote) {
                    if (line[i] == '\\' && i + 1 < line.size()) {
                        i++;
                    }
                    i++;
                }
                if (i < line.size()) {
                    source.code += quote;
                    i++;
                }
                continue;
            }
            source.code += line[i];
            i++;
        }
        lines->push_back(source);
    }
}

// `#define DEBUG_SET(...)` and the GYRO_FILTER_*_DEBUG_SET wrappers define the
// macro rather than write a field; the call sites that expand them are
// annotated where they sit.
bool definesDebugSetMacro(const std::string &code)
{
    const std::string text = trim(code);
    if (text.empty() || text[0] != '#') {
        return false;
    }
    size_t i = text.find("define");
    if (i == std::string::npos) {
        return false;
    }
    i += strlen("define");
    while (i < text.size() && isspace((unsigned char)text[i])) {
        i++;
    }
    const size_t nameStart = i;
    while (i < text.size() && isIdentifierChar(text[i])) {
        i++;
    }
    return endsWith(text.substr(nameStart, i - nameStart), DEBUG_SET_NAME);
}

struct CallSite {
    std::string file;
    int beginLine;          // 1 based
    int endLine;            // the line the call ends on, which carries the annotation
    std::string indexArg;   // the index argument as written
};

// Walks from the opening parenthesis to the one that closes the call, which may
// be several lines down, splitting the arguments on the way.
bool readCall(const std::vector<SourceLine> &lines, size_t startLine, size_t openParen,
              size_t *endLine, std::vector<std::string> *args)
{
    int depth = 0;
    std::string arg;
    args->clear();

    for (size_t line = startLine; line < lines.size(); line++) {
        const std::string &code = lines[line].code;
        for (size_t i = (line == startLine) ? openParen : 0; i < code.size(); i++) {
            const char c = code[i];
            if (c == '(') {
                depth++;
                if (depth == 1) {
                    continue;
                }
            } else if (c == ')') {
                depth--;
                if (depth == 0) {
                    args->push_back(trim(arg));
                    *endLine = line;
                    return true;
                }
            } else if (c == ',' && depth == 1) {
                args->push_back(trim(arg));
                arg.clear();
                continue;
            }
            arg += c;
        }
        arg += ' ';
    }
    return false;
}

void findCalls(const std::string &file, const std::vector<SourceLine> &lines, std::vector<CallSite> *calls)
{
    for (size_t line = 0; line < lines.size(); line++) {
        const std::string &code = lines[line].code;
        if (definesDebugSetMacro(code)) {
            continue;
        }
        size_t at = 0;
        while ((at = code.find(DEBUG_SET_NAME, at)) != std::string::npos) {
            size_t after = at + strlen(DEBUG_SET_NAME);
            if (after < code.size() && isIdentifierChar(code[after])) {
                at = after;
                continue;
            }
            size_t paren = after;
            while (paren < code.size() && isspace((unsigned char)code[paren])) {
                paren++;
            }
            if (paren >= code.size() || code[paren] != '(') {
                at = after;
                continue;
            }

            size_t endLine = line;
            std::vector<std::string> args;
            if (readCall(lines, line, paren, &endLine, &args)) {
                CallSite call;
                call.file = file;
                call.beginLine = (int)line + 1;
                call.endLine = (int)endLine + 1;
                // DEBUG_SET(mode, index, value) and the GYRO_FILTER_AXIS wrapper
                // that takes an axis in front of it both write the value last,
                // so the index is always the argument before it.
                call.indexArg = (args.size() >= 3) ? args[args.size() - 2] : "";
                calls->push_back(call);
            }
            at = after;
        }
    }
}

// The headers a file includes by name, which is how an [enum:...] type reaches
// the call site. Betaflight writes these relative to src/main.
void findIncludes(const std::vector<SourceLine> &lines, std::vector<std::string> *includes)
{
    for (size_t line = 0; line < lines.size(); line++) {
        if (!lines[line].include.empty()) {
            includes->push_back(lines[line].include);
        }
    }
}

void listSources(const std::string &directory, std::vector<std::string> *files)
{
    DIR *dir = opendir(directory.c_str());
    if (dir == NULL) {
        return;
    }
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        const std::string name = entry->d_name;
        if (name == "." || name == "..") {
            continue;
        }
        const std::string path = directory + "/" + name;
        struct stat info;
        if (stat(path.c_str(), &info) != 0) {
            continue;
        }
        if (S_ISDIR(info.st_mode)) {
            listSources(path, files);
        } else if (endsWith(name, ".c") || endsWith(name, ".h")) {
            files->push_back(path);
        }
    }
    closedir(dir);
}

// The test runs from src/test, but say where a checkout is rather than assume it.
std::string findSourceRoot(void)
{
    const char *candidates[] = { "../main", "src/main", "../../src/main" };
    for (size_t i = 0; i < ARRAY_LENGTH(candidates); i++) {
        struct stat info;
        const std::string header = std::string(candidates[i]) + "/build/debug.h";
        if (stat(header.c_str(), &info) == 0) {
            return candidates[i];
        }
    }
    return "";
}

// src/main/... reads better in a failure than ../main/...
std::string displayPath(const std::string &path)
{
    const size_t main = path.find("main/");
    return (main == std::string::npos) ? path : "src/" + path.substr(main);
}

std::string readFile(const std::string &path)
{
    std::ifstream file(path.c_str());
    std::ostringstream contents;
    contents << file.rdbuf();
    return contents.str();
}

class DebugAnnotations : public ::testing::Test {
public:
    static void SetUpTestCase(void)
    {
        sourceRoot = findSourceRoot();
        if (sourceRoot.empty()) {
            return;
        }

        std::vector<std::string> files;
        listSources(sourceRoot, &files);

        for (size_t i = 0; i < files.size(); i++) {
            std::vector<SourceLine> lines;
            readSource(files[i], &lines);

            std::vector<CallSite> fileCalls;
            findCalls(files[i], lines, &fileCalls);
            calls.insert(calls.end(), fileCalls.begin(), fileCalls.end());

            // The line a call ends on carries its annotation. Every other line
            // it spans - the one it starts on, and each one in between - is a
            // line an annotation is read from by nothing.
            std::set<int> endLines;
            std::set<int> continuationLines;
            for (size_t c = 0; c < fileCalls.size(); c++) {
                endLines.insert(fileCalls[c].endLine);
                for (int line = fileCalls[c].beginLine; line < fileCalls[c].endLine; line++) {
                    continuationLines.insert(line);
                }
            }

            for (size_t line = 0; line < lines.size(); line++) {
                if (!lines[line].hasAnnotation) {
                    continue;
                }
                const int lineNumber = (int)line + 1;
                annotations[FieldKey(files[i], lineNumber)] = lines[line].annotation;
                if (endLines.count(lineNumber)) {
                    continue;
                }
                // A `//!<` outside a call is somebody's doxygen member comment,
                // and none of this test's business.
                const bool onACall = lines[line].code.find(DEBUG_SET_NAME) != std::string::npos;
                if (onACall || continuationLines.count(lineNumber)) {
                    strayAnnotations.push_back(FieldKey(files[i], lineNumber));
                }
            }

            findIncludes(lines, &includesIn[files[i]]);

            // Every `} name;` the file defines, so an [enum:...] can be checked
            // against the types its call site can actually see.
            for (size_t line = 0; line < lines.size(); line++) {
                const std::string code = trim(lines[line].code);
                if (code.size() < 3 || code[0] != '}' || code[code.size() - 1] != ';') {
                    continue;
                }
                const std::string name = trim(code.substr(1, code.size() - 2));
                bool isIdentifier = !name.empty();
                for (size_t c = 0; c < name.size(); c++) {
                    isIdentifier = isIdentifier && isIdentifierChar(name[c]);
                }
                if (isIdentifier) {
                    typesIn[files[i]].insert(name);
                    if (!definedIn.count(name)) {
                        definedIn[name] = files[i];
                    }
                }
            }
        }
    }

    // What a file can see: what it defines, and what the headers it includes
    // define, all the way down. Betaflight writes an include relative to
    // src/main, so that is how one is resolved back to a file.
    static std::set<std::string> typesVisibleIn(const std::string &file)
    {
        std::set<std::string> visible;
        std::set<std::string> seen;
        std::vector<std::string> pending(1, file);

        while (!pending.empty()) {
            const std::string current = pending.back();
            pending.pop_back();
            if (!seen.insert(current).second) {
                continue;
            }
            const std::map<std::string, std::set<std::string> >::const_iterator types = typesIn.find(current);
            if (types != typesIn.end()) {
                visible.insert(types->second.begin(), types->second.end());
            }
            const std::map<std::string, std::vector<std::string> >::const_iterator includes = includesIn.find(current);
            if (includes == includesIn.end()) {
                continue;
            }
            for (size_t i = 0; i < includes->second.size(); i++) {
                const std::string path = sourceRoot + "/" + includes->second[i];
                if (typesIn.count(path) || includesIn.count(path)) {
                    pending.push_back(path);
                }
            }
        }
        return visible;
    }

    typedef std::pair<std::string, int> FieldKey;

    static std::string annotationAt(const CallSite &call)
    {
        const std::map<FieldKey, std::string>::const_iterator found =
            annotations.find(FieldKey(call.file, call.endLine));
        return (found == annotations.end()) ? "" : found->second;
    }

    static std::string sourceRoot;
    static std::vector<CallSite> calls;
    static std::map<FieldKey, std::string> annotations;
    static std::vector<FieldKey> strayAnnotations;
    static std::map<std::string, std::vector<std::string> > includesIn;
    static std::map<std::string, std::set<std::string> > typesIn;
    static std::map<std::string, std::string> definedIn;
};

std::string DebugAnnotations::sourceRoot;
std::vector<CallSite> DebugAnnotations::calls;
std::map<DebugAnnotations::FieldKey, std::string> DebugAnnotations::annotations;
std::vector<DebugAnnotations::FieldKey> DebugAnnotations::strayAnnotations;
std::map<std::string, std::vector<std::string> > DebugAnnotations::includesIn;
std::map<std::string, std::set<std::string> > DebugAnnotations::typesIn;
std::map<std::string, std::string> DebugAnnotations::definedIn;

} // namespace

/*
 * The grammar, checked against literal annotations.
 */

static std::string problemsWith(const std::string &annotation)
{
    Annotation parsed;
    const std::vector<std::string> errors = checkAnnotation(annotation, &parsed);
    std::string joined;
    for (size_t i = 0; i < errors.size(); i++) {
        joined += (joined.empty() ? "" : "; ") + errors[i];
    }
    return joined;
}

TEST(DebugAnnotationGrammar, AcceptsTheFourShapes)
{
    EXPECT_EQ("", problemsWith("Cycle Time [unit:us]"));
    EXPECT_EQ("", problemsWith("Failsafe Phase [enum:failsafePhase_e]"));
    EXPECT_EQ("", problemsWith("Frame Flags [flags:Channel 17|Channel 18|-|Failsafe]"));
    EXPECT_EQ("", problemsWith("Loop Iterations"));
}

TEST(DebugAnnotationGrammar, AcceptsAFactorASignAndADimensionlessScale)
{
    EXPECT_EQ("", problemsWith("Heading [unit:0.1deg]"));
    EXPECT_EQ("", problemsWith("Pressure [unit:100Pa]"));
    EXPECT_EQ("", problemsWith("Uplink RSSI [unit:-1dBm]"));
    EXPECT_EQ("", problemsWith("Throttle Ratio [unit:0.001]"));
    EXPECT_EQ("", problemsWith("Velocity Variance [unit:cm2/s2]"));
    EXPECT_NE(std::string::npos, problemsWith("Uplink RSSI [unit:-dBm]").find("with no factor"));
    EXPECT_NE(std::string::npos, problemsWith("Yaw Rate [unit:.dps]").find("with no factor"));
}

TEST(DebugAnnotationGrammar, RejectsAUnitSymbolNoConsumerKnows)
{
    EXPECT_NE(std::string::npos, problemsWith("Altitude P Term [unit:u]").find("unknown unit symbol 'u'"));
    EXPECT_NE(std::string::npos, problemsWith("Yaw Rate [unit:0.1deg/s]").find("unknown unit symbol 'deg/s'"));
    EXPECT_EQ("", problemsWith("Yaw Rate [unit:0.1dps]"));
}

TEST(DebugAnnotationGrammar, RejectsABracketThatNamesNothing)
{
    EXPECT_NE(std::string::npos, problemsWith("Cycle Time [us]").find("carries no key"));
    EXPECT_NE(std::string::npos, problemsWith("Cycle Time [units:us]").find("carries no key"));
    EXPECT_NE(std::string::npos, problemsWith("Cycle Time [unit:]").find("empty unit"));
}

TEST(DebugAnnotationGrammar, RequiresALabelWithoutBrackets)
{
    EXPECT_NE(std::string::npos, problemsWith("[unit:us]").find("no label"));
    EXPECT_NE(std::string::npos, problemsWith("").find("empty annotation"));
    EXPECT_NE(std::string::npos, problemsWith("Setpoint [HPF] [unit:dps]").find("contains a bracket"));
    EXPECT_EQ("", problemsWith("Setpoint HPF (roll) [unit:dps]"));
}

TEST(DebugAnnotationGrammar, ChecksTheIndexSpec)
{
    EXPECT_EQ("", problemsWith("[index:0..2] Gyro Filtered ({roll|pitch|yaw}) [unit:dps]"));
    EXPECT_EQ("", problemsWith("[index:0,2,4] Motor Output ({one|two|three}) [unit:%]"));
    EXPECT_NE(std::string::npos,
              problemsWith("[index:0..2] Gyro Filtered ({roll|pitch}) [unit:dps]").find("for 3 index(es)"));
    EXPECT_NE(std::string::npos,
              problemsWith("Gyro Filtered ({roll|pitch|yaw}) [unit:dps]").find("no [index:...] spec"));
    EXPECT_NE(std::string::npos, problemsWith("[index:2..0] Backwards").find("ends before it starts"));
    EXPECT_NE(std::string::npos, problemsWith("[index:axis] Computed").find("is not an index"));
    EXPECT_NE(std::string::npos, problemsWith("[index:] Unnamed").find("empty index spec"));
    EXPECT_NE(std::string::npos,
              problemsWith("[index:0..2] Gyro Filtered ({roll||yaw}) [unit:dps]").find("leaves one of its fields unnamed"));
    EXPECT_NE(std::string::npos, problemsWith("[index:0..1] Value {a|b}}").find("unmatched brace"));
    EXPECT_NE(std::string::npos, problemsWith("Value }").find("unmatched brace"));
    EXPECT_NE(std::string::npos, problemsWith("[index:0..1] Value {a|b} {c|d}").find("more than one"));
    EXPECT_NE(std::string::npos, problemsWith("Gyro Filtered [index:0..2]").find("goes in front of the label"));
}

TEST(DebugAnnotationGrammar, TellsAConstantIndexFromAComputedOne)
{
    EXPECT_TRUE(isCompileTimeIndex("3"));
    EXPECT_TRUE(isCompileTimeIndex("DEBUG_ESC_DATA_AGE"));
    EXPECT_TRUE(isCompileTimeIndex("FD_YAW"));
    EXPECT_FALSE(isCompileTimeIndex("axis"));
    EXPECT_FALSE(isCompileTimeIndex("motorIndex"));
    EXPECT_FALSE(isCompileTimeIndex("2 * axis + 1"));
    EXPECT_FALSE(isCompileTimeIndex(""));
}

TEST(DebugAnnotationGrammar, ChecksEnumsAndFlags)
{
    EXPECT_NE(std::string::npos, problemsWith("Failsafe Phase [enum:failsafePhase]").find("is not an enum type name"));
    EXPECT_NE(std::string::npos, problemsWith("Frame Flags [flags:]").find("empty flag list"));
    EXPECT_NE(std::string::npos, problemsWith("Frame Flags [flags:Signal Loss||Failsafe]").find("empty name"));
}

/*
 * The same grammar, over the firmware sources.
 */

TEST_F(DebugAnnotations, TheFirmwareSourcesAreFound)
{
    ASSERT_FALSE(sourceRoot.empty()) << "src/main was not found from the working directory; "
                                        "run the tests with 'make test' from the repository root";
    EXPECT_GT(calls.size(), 100u) << "found hardly any DEBUG_SET() calls, so the scan is broken";
}

TEST_F(DebugAnnotations, EveryDebugSetCallIsAnnotated)
{
    ASSERT_FALSE(sourceRoot.empty());

    for (size_t i = 0; i < calls.size(); i++) {
        if (annotationAt(calls[i]).empty()) {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << "this DEBUG_SET() has no //!< annotation, so no tool can say what the field means.\n"
                << "See the grammar above DEBUG_SET() in src/main/build/debug.h.";
        }
    }
}

TEST_F(DebugAnnotations, EveryAnnotationFollowsTheGrammar)
{
    ASSERT_FALSE(sourceRoot.empty());

    for (size_t i = 0; i < calls.size(); i++) {
        const std::string annotation = annotationAt(calls[i]);
        if (annotation.empty()) {
            continue;
        }
        Annotation parsed;
        const std::vector<std::string> errors = checkAnnotation(annotation, &parsed);
        for (size_t e = 0; e < errors.size(); e++) {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << "//!< " << annotation << "\n" << errors[e];
        }
    }
}

TEST_F(DebugAnnotations, EveryAnnotationSitsOnTheLineItsCallEndsOn)
{
    ASSERT_FALSE(sourceRoot.empty());

    for (size_t i = 0; i < strayAnnotations.size(); i++) {
        ADD_FAILURE_AT(displayPath(strayAnnotations[i].first).c_str(), strayAnnotations[i].second)
            << "an annotation on a DEBUG_SET() that continues onto another line is read by nothing: "
            << "move it to the line the call ends on.";
    }
}

TEST_F(DebugAnnotations, AnEnumShapeNamesATypeTheFirmwareDefines)
{
    ASSERT_FALSE(sourceRoot.empty());

    for (size_t i = 0; i < calls.size(); i++) {
        Annotation parsed;
        checkAnnotation(annotationAt(calls[i]), &parsed);
        if (parsed.enumType.empty() || typesVisibleIn(calls[i].file).count(parsed.enumType)) {
            continue;
        }

        const std::map<std::string, std::string>::const_iterator elsewhere = definedIn.find(parsed.enumType);
        if (elsewhere == definedIn.end()) {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << "no 'typedef enum { ... } " << parsed.enumType << ";' in src/main: "
                << "tooling reads the enumerator names from the type, so it has to exist.";
        } else {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << parsed.enumType << " is defined in " << displayPath(elsewhere->second)
                << ", which this file does not include: tooling reads the enumerator names from where the "
                << "call site can see them.";
        }
    }
}

TEST_F(DebugAnnotations, AnIndexSpecIsGivenExactlyWhenTheIndexIsComputedAtRunTime)
{
    ASSERT_FALSE(sourceRoot.empty());

    for (size_t i = 0; i < calls.size(); i++) {
        const std::string annotation = annotationAt(calls[i]);
        if (annotation.empty()) {
            continue;
        }
        const bool constant = isCompileTimeIndex(calls[i].indexArg);
        const bool specified = startsWith(annotation, "[index:");

        if (constant && specified) {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << "the call writes debug[" << calls[i].indexArg << "], a compile-time constant that tooling "
                << "reads from the call itself: drop the [index:...] spec.";
        } else if (!constant && !specified) {
            ADD_FAILURE_AT(displayPath(calls[i].file).c_str(), calls[i].endLine)
                << "no static scan can evaluate the index '" << calls[i].indexArg << "', so say what the call "
                << "writes: [index:2], [index:0..2] or [index:0,2,4]. A constant is written in capitals.";
        }
    }
}

TEST_F(DebugAnnotations, EveryAcceptedUnitIsListedInTheHeader)
{
    ASSERT_FALSE(sourceRoot.empty());

    const std::string header = readFile(sourceRoot + "/build/debug.h");
    std::set<std::string> words;
    std::istringstream stream(header);
    std::string word;
    const std::string punctuation = ",.:;()[]";
    while (stream >> word) {
        while (!word.empty() && punctuation.find(word[word.size() - 1]) != std::string::npos) {
            word.erase(word.size() - 1);
        }
        while (!word.empty() && punctuation.find(word[0]) != std::string::npos) {
            word.erase(0, 1);
        }
        words.insert(word);
    }

    for (size_t i = 0; i < ARRAY_LENGTH(UNIT_SYMBOLS); i++) {
        EXPECT_TRUE(words.count(UNIT_SYMBOLS[i]) != 0)
            << "'" << UNIT_SYMBOLS[i] << "' is accepted here but is not in the grammar in src/main/build/debug.h";
    }
}

/*
 * A mode with no name is the same class of mistake: it leaves a NULL in the
 * CLI and CMS lookup table built from debugModeNames[].
 */

TEST(DebugModeNames, EveryDebugModeIsNamed)
{
    ASSERT_EQ((size_t)DEBUG_COUNT, ARRAY_LENGTH(debugModeNames));

    for (int mode = 0; mode < DEBUG_COUNT; mode++) {
        EXPECT_TRUE(debugModeNames[mode] != NULL)
            << "debugType_e value " << mode << " has no entry in debugModeNames[] in src/main/build/debug.c";
    }
}
