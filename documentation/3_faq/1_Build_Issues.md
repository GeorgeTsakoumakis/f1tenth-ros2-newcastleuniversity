# Build Issues

## Issue 1: colcon build has no effect

You might be noticing that after rebuilding your workspace after changing some configuration file, you are observing no changes. In this case, you need to delete the `build` and `install` directories within your workspace and rebuild the project. This is called a "clean build". You might find an alias for that called "clean_build" on the Newcastle University cars inside the `.bashrc` file.

## Issue 2: colcon build takes way too long to run

Unless you are installing Autoware and rebuilding, which takes about 6-7 hours (note: currently Autoware was removed from our cars), then in case you have only modified one package you can run the following command which will only rebuild that specific package

`colcon build --packages-select <package_name>`