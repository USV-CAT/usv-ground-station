# Voice packs

The spoken alerts are phrases from the **EdgeTX SD-card sound packs**,
<https://github.com/EdgeTX/edgetx-sdcard-sounds>, licensed under the GNU
General Public License v2.0 (see `LICENSE` in this folder). They are kept here
as separate files, unchanged, and are not compiled into the station.

- `en/` — the `SOUNDS/en` voice
- `ru/` — the `SOUNDS/ru` voice

Fetch them again, or another English voice, with:

```bash
./run.sh sounds                          # en and ru
./run.sh sounds --english-voice en_us-sara
```
