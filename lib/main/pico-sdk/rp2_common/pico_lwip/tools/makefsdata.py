#!/usr/bin/env python3
import argparse
import mimetypes
from pathlib import Path
import re

response_types = {
  200: "HTTP/1.0 200 OK",
  400: "HTTP/1.0 400 Bad Request",
  404: "HTTP/1.0 404 File not found",
  501: "HTTP/1.0 501 Not Implemented",
}

PAYLOAD_ALIGNMENT = 4
HTTPD_SERVER_AGENT = "lwIP/2.2.0d (http://savannah.nongnu.org/projects/lwip)"
LWIP_HTTPD_SSI_EXTENSIONS = [".shtml", ".shtm", ".ssi", ".xml", ".json"]

def process_file(input_dir, file):
    results = []

    # Check content type
    content_type, _ = mimetypes.guess_type(file)
    if content_type is None:
        content_type = "application/octet-stream"

    # file name
    data = f"/{file.relative_to(input_dir)}\x00"
    comment = f"\"/{file.relative_to(input_dir)}\" ({len(data)} chars)"
    while(len(data) % PAYLOAD_ALIGNMENT != 0):
        data += "\x00"
    results.append({'data': bytes(data, "utf-8"), 'comment': comment});

    # Header
    response_type = 200
    for response_id in response_types:
        if file.name.startswith(f"{response_id}."):
            response_type = response_id
            break
    data = f"{response_types[response_type]}\r\n"
    comment = f"\"{response_types[response_type]}\" ({len(data)} chars)"
    results.append({'data': bytes(data, "utf-8"), 'comment': comment});

    # user agent
    data = f"Server: {HTTPD_SERVER_AGENT}\r\n"
    comment = f"\"Server: {HTTPD_SERVER_AGENT}\" ({len(data)} chars)"
    results.append({'data': bytes(data, "utf-8"), 'comment': comment});

    if file.suffix not in LWIP_HTTPD_SSI_EXTENSIONS:
        # content length
        file_size = file.stat().st_size
        data = f"Content-Length: {file_size}\r\n"
        comment = f"\"Content-Length: {file_size}\" ({len(data)} chars)"
        results.append({'data': bytes(data, "utf-8"), 'comment': comment});

    # content type
    data = f"Content-Type: {content_type}\r\n\r\n"
    comment = f"\"Content-Type: {content_type}\" ({len(data)} chars)"
    results.append({'data': bytes(data, "utf-8"), 'comment': comment});

    # file contents
    data = file.read_bytes()
    comment = f"raw file data ({len(data)} bytes)"
    results.append({'data': data, 'comment': comment});

    return results;

def process_file_list(fd, input):
    data = []
    fd.write("#include \"lwip/apps/fs.h\"\n")
    fd.write("\n")
    # generate the page contents
    input_dir = None
    for name in input:
        file = Path(name)
        if not file.is_file():
            raise RuntimeError(f"File not found: {name}")
        # Take the input directory from the first file
        if input_dir is None:
            input_dir = file.parent
        results = process_file(input_dir, file)

        # make a variable name
        var_name = str(file.relative_to(input_dir))
        var_name = re.sub(r"\W+", "_", var_name, flags=re.ASCII)

        # Add a suffix if the variable name is used already
        if any(d["data_var"] == f"data_{var_name}" for d in data):
            var_name += f"_{len(data)}"

        data_var = f"data_{var_name}"
        file_var = f"file_{var_name}"

        # variable containing the raw data
        fd.write(f"static const unsigned char {data_var}[] = {{\n")
        for entry in results:
            fd.write(f"\n    /* {entry['comment']} */\n")
            byte_count = 0;
            for b in entry['data']:
                if byte_count % 16 == 0:
                    fd.write("    ")
                byte_count += 1
                fd.write(f"0x{b:02x},")
                if byte_count % 16 == 0:
                    fd.write("\n")
            if byte_count % 16 != 0:
                fd.write("\n")
        fd.write(f"}};\n\n")

        # set the flags
        flags = "FS_FILE_FLAGS_HEADER_INCLUDED"
        if file.suffix not in LWIP_HTTPD_SSI_EXTENSIONS:
            flags += " | FS_FILE_FLAGS_HEADER_PERSISTENT"
        else:
            flags += " | FS_FILE_FLAGS_SSI"

        # add variable details to the list
        data.append({'data_var': data_var, 'file_var': file_var, 'name_size': len(results[0]['data']), 'flags': flags})

    # generate the page details
    last_var = "NULL"
    for entry in data:
        fd.write(f"const struct fsdata_file {entry['file_var']}[] = {{{{\n")
        fd.write(f"    {last_var},\n")
        fd.write(f"    {entry['data_var']},\n")
        fd.write(f"    {entry['data_var']} + {entry['name_size']},\n")
        fd.write(f"    sizeof({entry['data_var']}) - {entry['name_size']},\n")
        fd.write(f"    {entry['flags']},\n")
        fd.write(f"}}}};\n\n")
        last_var = entry['file_var']
    fd.write(f"#define FS_ROOT {last_var}\n")
    fd.write(f"#define FS_NUMFILES {len(data)}\n")

def run_tool():
    parser = argparse.ArgumentParser(prog="makefsdata.py", description="Generates a source file for the lwip httpd server")
    parser.add_argument(
        "-i",
        "--input",
        help="input files to add as http content",
        required=True,
        nargs='+'
    )
    parser.add_argument(
        "-o",
        "--output",
        help="name of the source file to generate",
        required=True,
    )
    args = parser.parse_args()
    print(args.input)

    mimetypes.init()
    for ext in [".shtml", ".shtm", ".ssi"]:
        mimetypes.add_type("text/html", ext)

    with open(args.output, "w") as fd:
        process_file_list(fd, args.input)

if __name__ == "__main__":
    run_tool()
