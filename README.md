# Obelisk: A Stable Robot Control Interface
This repository hosts generic interfaces for controlling the lab's robots with ROS2.

[Docs](https://caltech-amber.github.io/obelisk/)

## Development
We recommend developing locally inside a `conda` environment for `sphinx` integration. To set up the development workflow, run the following commands from the repository root
```
# for system-wide dependencies
bash dev_setup.sh

# for python dependencies (you can choose different dependency groups besides dev, see pyproject.toml)
conda env create -f environment.yml --name <my_env>
conda activate <my_env>
pip install -e .[dev]
```

### Building Docs
In the repository root, to build the docs locally, run `sphinx-build -M html docs/source/ docs/build/`.
