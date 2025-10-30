# Obelisk Documentation

This directory contains the Sphinx documentation for Obelisk.

## Building the Documentation

### Setup with Conda

1. Create the conda environment:
```bash
conda env create -f environment.yml
```

2. Activate the environment:
```bash
conda activate obelisk-docs
```

3. Build the documentation:
```bash
cd docs
sphinx-build -b html source build/html
```

4. View the documentation by opening `build/html/index.html` in your browser.

### Updating the Environment

If the environment.yml file is updated, you can update your existing environment:
```bash
conda env update -f environment.yml --prune
```

### Clean Build

To do a clean rebuild of the documentation:
```bash
rm -rf build/
sphinx-build -b html source build/html
```
