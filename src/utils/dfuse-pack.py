#!/usr/bin/python

# Written by Antonio Galea - 2010/11/18
# Distributed under Gnu LGPL 3.0
# see http://www.gnu.org/licenses/lgpl-3.0.txt
#
# based on a modified version of this script from https://sourceforge.net/p/dfu-util/tickets/35/#357c
# with the patch supplied in https://sourceforge.net/p/dfu-util/tickets/35/#a2b6

import sys,struct,zlib,os
import binascii
from optparse import OptionParser
from intelhex import IntelHex

DEFAULT_DEVICE="0x0483:0xdf11"
DEFAULT_NAME=b'ST...'

def named(tuple,names):
  return dict(list(zip(names.split(),tuple)))
def consume(fmt,data,names):
  n = struct.calcsize(fmt)
  return named(struct.unpack(fmt,data[:n]),names),data[n:]
def cstring(bytestring):
  return bytestring.partition(b'\0')[0]
def compute_crc(data):
  return 0xFFFFFFFF & -zlib.crc32(data) -1

def parse(file,dump_images=False):
  print('File: "%s"' % file)
  data = open(file,'rb').read()
  crc = compute_crc(data[:-4])
  prefix, data = consume('<5sBIB',data,'signature version size targets')
  print('%(signature)s v%(version)d, image size: %(size)d, targets: %(targets)d' % prefix)
  for t in range(prefix['targets']):
    tprefix, data  = consume('<6sBI255s2I',data,'signature altsetting named name size elements')
    tprefix['num'] = t
    if tprefix['named']:
      tprefix['name'] = cstring(tprefix['name'])
    else:
      tprefix['name'] = ''
    print('%(signature)s %(num)d, alt setting: %(altsetting)s, name: "%(name)s", size: %(size)d, elements: %(elements)d' % tprefix)
    tsize = tprefix['size']
    target, data = data[:tsize], data[tsize:]
    for e in range(tprefix['elements']):
      eprefix, target = consume('<2I',target,'address size')
      eprefix['num'] = e
      print('  %(num)d, address: 0x%(address)08x, size: %(size)d' % eprefix)
      esize = eprefix['size']
      image, target = target[:esize], target[esize:]
      if dump_images:
        out = '%s.target%d.image%d.bin' % (file,t,e)
        open(out,'wb').write(image)
        print('    DUMPED IMAGE TO "%s"' % out)
    if len(target):
      print("target %d: PARSE ERROR" % t)
  suffix = named(struct.unpack('<4H3sBI',data[:16]),'device product vendor dfu ufd len crc')
  print('usb: %(vendor)04x:%(product)04x, device: 0x%(device)04x, dfu: 0x%(dfu)04x, %(ufd)s, %(len)d, 0x%(crc)08x' % suffix)
  if crc != suffix['crc']:
    print("CRC ERROR: computed crc32 is 0x%08x" % crc)
  data = data[16:]
  if data:
    print("PARSE ERROR")

def checkbin(binfile):
  data = open(binfile,'rb').read()
  if (len(data) < 16):
    return
  crc = compute_crc(data[:-4])
  suffix = named(struct.unpack('<4H3sBI',data[-16:]),'device product vendor dfu ufd len crc')
  if crc == suffix['crc'] and suffix['ufd'] == b'UFD':
    print('usb: %(vendor)04x:%(product)04x, device: 0x%(device)04x, dfu: 0x%(dfu)04x, %(ufd)s, %(len)d, 0x%(crc)08x' % suffix)
    print("It looks like the file %s has a DFU suffix!" % binfile)
    print("Please remove any DFU suffix and retry.")
    sys.exit(1)

def build(file,targets,name=DEFAULT_NAME,device=DEFAULT_DEVICE):
  data = b''
  for t,target in enumerate(targets):
    tdata = b''
    for image in target:
      tdata += struct.pack('<2I',image['address'],len(image['data']))+image['data']
    tdata = struct.pack('<6sBI255s2I',b'Target',0,1,name,len(tdata),len(target)) + tdata
    data += tdata
  data  = struct.pack('<5sBIB',b'DfuSe',1,len(data)+11,len(targets)) + data
  v,d=[int(x,0) & 0xFFFF for x in device.split(':',1)]
  data += struct.pack('<4H3sB',0x2200,d,v,0x011a,b'UFD',16)
  crc   = compute_crc(data)
  data += struct.pack('<I',crc)
  open(file,'wb').write(data)

if __name__=="__main__":
  usage = """
%prog [-d|--dump] infile.dfu
%prog {-b|--build} address:file.bin [-b address:file.bin ...] [{-D|--device}=vendor:device] outfile.dfu
%prog {-s|--build-s19} file.s19 [{-D|--device}=vendor:device] outfile.dfu
%prog {-i|--ihex} file.hex [-i file.hex ...] [{-D|--device}=vendor:device] outfile.dfu"""
  parser = OptionParser(usage=usage)
  parser.add_option("-b", "--build", action="append", dest="binfiles",
    help="build a DFU file from given BINFILES. Note that the BINFILES must not have any DFU suffix!", metavar="BINFILES")
  parser.add_option("-i", "--ihex", action="append", dest="hexfiles",
    help="build a DFU file from given HEXFILES", metavar="HEXFILES")
  parser.add_option("-s", "--build-s19", type="string", dest="s19files",
    help="build a DFU file from given S19 S-record file.", metavar="S19FILE")
  parser.add_option("-D", "--device", action="store", dest="device",
    help="build for DEVICE, defaults to %s" % DEFAULT_DEVICE, metavar="DEVICE")
  parser.add_option("-d", "--dump", action="store_true", dest="dump_images",
    default=False, help="dump contained images to current directory")
  (options, args) = parser.parse_args()

  if (options.binfiles or options.hexfiles) and len(args)==1:
    target = []

    if options.binfiles:
      for arg in options.binfiles:
        try:
          address,binfile = arg.split(':',1)
        except ValueError:
          print("Address:file couple '%s' invalid." % arg)
          sys.exit(1)
        try:
          address = int(address,0) & 0xFFFFFFFF
        except ValueError:
          print("Address %s invalid." % address)
          sys.exit(1)
        if not os.path.isfile(binfile):
          print("Unreadable file '%s'." % binfile)
          sys.exit(1)
        checkbin(binfile)
        target.append({ 'address': address, 'data': open(binfile,'rb').read() })

    if options.hexfiles:
      for hex in options.hexfiles:
        ih = IntelHex(hex)
        for (address,end) in ih.segments():
          try:
            address = address & 0xFFFFFFFF
          except ValueError:
            print("Address %s invalid." % address)
            sys.exit(1)
          target.append({ 'address': address, 'data': ih.tobinstr(start=address, end=end-1)})

    outfile = args[0]
    device = DEFAULT_DEVICE
    if options.device:
      device=options.device
    try:
      v,d=[int(x,0) & 0xFFFF for x in device.split(':',1)]
    except:
      print("Invalid device '%s'." % device)
      sys.exit(1)
    build(outfile,[target],DEFAULT_NAME,device)
  elif options.s19files and len(args)==1:
    target = []
    name = DEFAULT_NAME
    data_record_count = 0
    with open(options.s19files) as f:
      for number, line in enumerate(f, 1):
        line = line.strip()
        if not line:
          continue
        record = line[:2]
        if record not in ('S0', 'S1', 'S2', 'S3', 'S5', 'S6', 'S7', 'S8', 'S9'):
          parser.error("line %d: unknown S-record type" % number)
        try:
          raw = binascii.unhexlify(line[2:])
        except (ValueError, binascii.Error):
          parser.error("line %d: invalid S-record hex" % number)
        address_size = {'S0': 2, 'S1': 2, 'S2': 3, 'S3': 4, 'S5': 2,
                        'S6': 3, 'S7': 4, 'S8': 3, 'S9': 2}[record]
        if len(raw) < address_size + 2 or raw[0] != len(raw) - 1:
          parser.error("line %d: invalid S-record byte count" % number)
        if sum(raw) & 0xff != 0xff:
          parser.error("line %d: invalid S-record checksum" % number)
        data = raw[1 + address_size:-1]
        if record == 'S0':
          name = data[:-4] if data.endswith(b'.s19') else data
        elif record in ('S1', 'S2', 'S3'):
          data_record_count += 1
          if not data:
            continue
          address = int.from_bytes(raw[1:1 + address_size], 'big')
          if address + len(data) > 0x100000000:
            parser.error("line %d: S-record data exceeds the 32-bit address space" % number)
          if target and target[-1]['address'] + len(target[-1]['data']) == address:
            target[-1]['data'].extend(data)
          else:
            target.append({'address': address, 'data': bytearray(data)})
        else:
          if data:
            parser.error("line %d: S-record control record must not contain data" % number)
          if record in ('S5', 'S6'):
            declared_count = int.from_bytes(raw[1:1 + address_size], 'big')
            if declared_count != data_record_count:
              parser.error("line %d: S-record data record count does not match" % number)
    if not target:
      parser.error("no S-record image data")
    outfile = args[0]
    device = DEFAULT_DEVICE
    if options.device:
      device=options.device
    try:
      v,d=[int(x,0) & 0xFFFF for x in device.split(':',1)]
    except:
      print("Invalid device '%s'." % device)
      sys.exit(1)
    build(outfile,[target],name,device)
  elif len(args)==1:
    infile = args[0]
    if not os.path.isfile(infile):
      print("Unreadable file '%s'." % infile)
      sys.exit(1)
    parse(infile, dump_images=options.dump_images)
  else:
    parser.print_help()
    sys.exit(1)
