# Contributing

Thanks for your interest. This is a personal research project, so the scope is narrow, but issues and small pull requests are welcome.

## Issues
- Bugs: say what you ran, what you expected and what happened, with versions (Python, MuJoCo, OS).
- Questions about the method or results: open an issue and link the evidence page you are asking about.

## Pull requests
1. Keep each change small and focused.
2. Run the checks before opening it:
   ```bash
   python -m pip install -e ".[publication]" pytest
   python publish_check.py
   python -m pytest -q -p no:cacheprovider
   ```
   For the firmware: `cmake -S firmware -B build && cmake --build build && ctest --test-dir build`.
3. Keep upstream credit lines and license notices unchanged (see [NOTICE.md](NOTICE.md)).
4. Do not add CAD meshes, datasets, real-robot logs or personal data.

By contributing you agree that your contribution is licensed under the repository's [MIT License](LICENSE).
