# Recorded approach in the scene twin

**FROM FILE.** The MuJoCo display replays detector packets from the lab approach. The source receipt records 649 checked poses for the approach and a 300-frame render made later in a graphical session. The original geometry, inferred motion and advisory colours are preserved in the [animation](../../media/scene-twin-approach.gif). [Source replay receipt](https://github.com/KaiserRichard/wbr-perception-phase1-runtime/blob/fde5721af0627eaf60c78d95296ab26924425e37/docs/scene_twin/RUN_RECEIPT.md)

**FROM FILE.** The first valid observation anchors the inferred motion. For the approach, forward displacement follows the change in filtered range, with an assumed camera pose. Blue marks a jump candidate, red marks a stop candidate and green marks `NONE`. Run the viewer on a supplied packet directory from `perception/`.

```bash
python3 -m tools.scene_twin.view --run /path/to/recorded-approach
```

**UNKNOWN.** True robot displacement and jump feasibility require independent measurements.
