# Parameter Groups

## Introduction
'Parameter Groups' is the name of the pattern that is used in Betaflight to store (and at a later point transfer) firmware configuration data.

With parameter groups, parameters are combined in logical groups (e.g. all the parameters for one particular device / feature go into one parameter group). These groups are modelled as data structures in the firmware.

## Rules

The following rules have to be taken into consideration when dealing with parameter groups in code:

1. new parameters should always be appended at the end of the parameter group. If this is the only change to the parameter group, there are no further changes required. Previously stored versions of the data in this parameter group can still be read. New elements will be initialised to 0 in this case;
2. any changes to a parameter group that are not covered in 1. (e.g. removing elements, changing the type of elements), the version of the parameter group (as defined in `PG_REGISTER...`) has to be incremented in order to make the code handling parameter groups aware that the format of the parameter group has changed and previously stored versions of it are now invalid;
3. all changes to parameter group arrays (declared with `PG_DECLARE_ARRAY()`) or arrays contained within the struct of a parameter group require that the version of the parameter group (as defined in `PG_REGISTER..` has to be incremented as well, and the same considerations as for 2. apply;
4. When creating a new parameter group, the `PG_DECLARE()` / `PG_DECLARE_ARRAY()` shall be placed right after the definition of the struct used as the type of the parameter group in the code. This serves the purpose of making it easy for developers to determine when a struct they are working on is used in a parameter group and subject to these rules;
5. Any struct that is used as an element in a struct that is used in a parameter group (directly or indirectly) is subject to constraints 2. and 3. above. In order to make it possible for developers to determine that this is the case, a comment `// Used in parameter group <group name>` shall be added after the definition of the struct for every parameter group that the struct is used in.
