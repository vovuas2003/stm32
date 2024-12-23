0. Pins, timers, clock settings and anything else.
1. Pinout & Configuration -> System Core -> SYS -> Debug -> Serial Wire.
2.1. Project Manager -> Project -> Project Name and Project Location -> what we want (in "location" will be automatically created folder "name")
2.2. Project Manager -> Project -> Toolchain/IDE -> Makefile
2.3. Project Manager -> Project -> Firmware Relative Path (DON'T "Use Default Firmware Location") -> path\to\STM32Cube_FW_F1_V1.8.5 (for example)
2.4. Project Manager -> Code Generator -> Add necessary library files as reference in the toolchain project configuration file
3. Generate code