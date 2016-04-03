# Configuration Format

The configuration format and external protocol use some of the same concepts
as SAE J1939.  A parameter group (PG) is a set of parameters belonging
to the same topic and are stored and sent together.  A parameter group
instance has a unique parameter group number (PGN).  Each parameter
also has a suspect parameter number (SPN) which can be used to get or
set a parameter directly.

When used as on-wire formats structures should be packed to give the
same in storage, in memory, and on-wire format.  However care must be taken
over memory alignment issues when packing structures.

The PGs can be defined on a system-wide basis on a profile specific basis.
profiles can be activated on the fly.

The storage consists of a header, zero or more PGs, a footer and a checksum.
To keep the RAM usage low, the parameters are written directly to flash
which means that things that are only known at the end, such as the
size are stored in the footer. The checksum is written after the footer.

The header holds:

* The format number.  This is bumped on incompatible changes.

Each stored PG holds:

* The size of this record
* The PGN
* Version number
* Profile number
* Flags
* The record format.  This is bumped on incompatible changes.
* The PG data

The footer holds:

* A zero to mark the end of data

The checksum is based on the header, PGs, and footer.

The PG registrations hold similar but not identical information (e.g. the profile
number is not known until it is stored).

## Initialiion function.

All fields are reset to 0 upon initialisation and then if a reset function is
defined for the group then initial settings can be defined by the system.

## Upgrading

Upgrades are done at the PG level and are detected by a difference in
size or version.  New fields can be added to the end of the parameter group.
The reset and initialisation function is called before upgrading so new
fields will first be reset to 0 and then initialised by the system if defined.

Note: Currently the code does not check the version field.
 
## Downgrading

Downgrades are done at the PG level.  Any trailing, unrecognised
fields will be silently dropped on load.  Saving the config back to
flash will discard these unrecognised fields.

## Incompatible changes

An incompatible change is where a field is inserted, deleted from the
middle, reordered, resized (including changing the size of a contained array),
or has the meaning changed.  Such changes should be handled by bumping the 
PG version field or allocating a new PGN.
