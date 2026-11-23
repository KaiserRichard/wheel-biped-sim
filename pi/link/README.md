**Tóm tắt [COMPUTED].** Giao tiếp Pi đã chạy lại và đạt 19/19 nhóm kiểm thử, gồm 8 nhóm Python và 11 nhóm đối chiếu C.

# Pi USB link

[FROM FILE] This snapshot is the corrected package from the supplied Pi-link workspace. The wire header is `<BBBBIQH>` (18 bytes); the short twist payload is 48 bytes, and the full payload is 56 bytes. These match the included C reference and the Python struct definitions. The package includes CRC32C, COBS framing, message codecs, streaming decode, rate gating and transport helpers.

[COMPUTED] During staging, the C reference was rebuilt as a Windows shared library and `tests/run_all.py` passed all 19 suites. The upstream correction receipt records the same count. Builds are omitted from publication.

## Reproduce

```sh
cmake -S c_ref_build -B build
cmake --build build
python tests/run_all.py
```

[FROM FILE] The cross tests accept the platform library in `build/`: a Linux shared object, Windows DLL or macOS dynamic library. Run without Python optimization so assertions remain active.

## What this teaches

[STANDARD PRACTICE] A struct format can have the correct field types but the wrong field count. Cross-language byte comparisons expose wire-layout drift before a serial device is connected.

## What the real robot must confirm

[UNKNOWN] USB latency, reconnect behavior, target timestamps and command watchdog timing must be measured with the actual controller. Tests here use host transports and the included C reference.
