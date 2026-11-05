---
name: colcon-build-recovery
description: Fix a failing colcon build from its output - missing dependencies, CMakeLists/package.xml/setup.py mistakes, interface generation - keeping every file consistent.
---

# colcon build recovery

1. Run `colcon_build` (or `colcon_diagnose` on a pasted log). Fix the **first** root cause only; later errors are often cascades.
2. Missing dependency: change **all three** together: `package.xml` (`<depend>`), `CMakeLists.txt` (`find_package` +
   `ament_target_dependencies`/`target_link_libraries`), then `rosdep install --from-paths src --ignore-src -y` in the build target.
3. Python packages (ament_python): `setup.py` `packages=[...]` must match the folder; entry points `pkg.module:main`;
   install launch/config with `data_files`; add `<exec_depend>` for runtime imports.
4. Interfaces: `rosidl_generate_interfaces(${PROJECT_NAME} ... DEPENDENCIES <msg pkgs>)`, `<member_of_group>rosidl_interface_packages</member_of_group>`.
5. Stale state after renames: remove `build/<pkg>` and `install/<pkg>` for that package only.
6. Rebuild just the package (`--packages-select`), then `--packages-up-to` its dependents.

Never "fix" by deleting the whole workspace or editing files under `install/`.
