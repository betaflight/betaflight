Configuration Format
====================

The configuration format and external protocol use the same concepts
as SAE J1939.  A parameter group (PG) is a set of parameters belonging
to the same topic and are stored and sent together.  A parameter group
instance has a unique parameter group number (PGN).  Each parameter
also has a suspect parameter number (SPN) which can be used to get or
set a parameter directly.

All structures are packed to give the same in storage, in memory, and
on-wire format.

The storage consists of a header, zero or more PGs, and a footer.  To
keep the RAM usage low, the parameters are written directly to flash
which means that things that are only known at the end, such as the
size and checksum, are stored in the footer.

The header holds:

* The format number.  This is bumped on incompatible changes.

A PG holds:

* The size of this record
* The PGN
* The record format.  This is bumped on incompatible changes.
* The PG data

The footer holds:

* A zero to mark the end of data
* The checksum across the header, PGs, and footer

Upgrading
=========
Upgrades are done at the PG level and are detected by a difference in
size.  New fields can be added to the end of the parameter group and
will be automatically filled in zero if missing.

Downgrading
===========
Downgrades are done at the PG level.  Any trailing, unrecognised
fields will be silently dropped on load.  Saving the config back to
flash will discard these unrecognised fields.

Incompatible changes
====================
An incompatible change is where a field is inserted, deleted from the
middle, reordered, resized, or has the meaning changed.  Such changes
should be handled by allocating a new PGN (preferred) or bumping the
PG level format field.
