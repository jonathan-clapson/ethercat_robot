- WAGO_750_354.xml
  Contains coupler and IOM information.
  Supported coupler firmware versions:
  - V01.01.08
  - V01.01.09
  - V01.01.10

- WAGO_750_354_no_modules.xml:
  Contains coupler information only. This file may be used with masters having problems processing the XML file containing IOM information.
  Supported coupler firmware versions:
  - V01.01.08
  - V01.01.09
  - V01.01.10

-----------------
Usage in TwinCAT:
-----------------
- Copy the desired XML file into TwinCAT's XML folder: \TwinCAT\IO\EtherCAT
- Make sure, that both files are not inside TwinCAT's configuration folder at the same time.
  TwinCAT's behavior is undefined if both files are in the XML folder at the same time.
- Online configuration:
  TwinCAT (at least V.2.11) will not complain if a IOM is found, which is currently not part of the XML file. In this case,
  either a Sync Manager configuration error will occurr, or your process data is screwed up.
- Offline configuration:
  TwinCAT (at least V.2.11) does not safely compare an offline configuration against the connected modules. A difference may not
  be detected. In this case, either a Sync Manager configuration error will occurr, or process data is screwed up.
- When using WAGO_750_354.xml:
  The PDO Index Assignment Workaround must be off (Object 0x2100:02 = FALSE)
- When using WAGO_750-354_online_only.xml:
  The PDO Index Assignment Workaroung must be on (Object 0x2100:02 = TRUE)
