# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/unita/erp_final_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unita/erp_final_ws/build

# Utility rule file for velodyne_pointcloud_gencfg.

# Include the progress variables for this target.
include velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/progress.make

velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud/cfg/TransformNodeConfig.py


/home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h: /home/unita/erp_final_ws/src/velodyne/velodyne_pointcloud/cfg/TransformNode.cfg
/home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.py.template
/home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h: /opt/ros/noetic/share/dynamic_reconfigure/templates/ConfigType.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/unita/erp_final_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating dynamic reconfigure files from cfg/TransformNode.cfg: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h /home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud/cfg/TransformNodeConfig.py"
	cd /home/unita/erp_final_ws/build/velodyne/velodyne_pointcloud && ../../catkin_generated/env_cached.sh /home/unita/erp_final_ws/build/velodyne/velodyne_pointcloud/setup_custom_pythonpath.sh /home/unita/erp_final_ws/src/velodyne/velodyne_pointcloud/cfg/TransformNode.cfg /opt/ros/noetic/share/dynamic_reconfigure/cmake/.. /home/unita/erp_final_ws/devel/share/velodyne_pointcloud /home/unita/erp_final_ws/devel/include/velodyne_pointcloud /home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud

/home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.dox: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.dox

/home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig-usage.dox: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig-usage.dox

/home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud/cfg/TransformNodeConfig.py: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud/cfg/TransformNodeConfig.py

/home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.wikidoc: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
	@$(CMAKE_COMMAND) -E touch_nocreate /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.wikidoc

velodyne_pointcloud_gencfg: velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg
velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/include/velodyne_pointcloud/TransformNodeConfig.h
velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.dox
velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig-usage.dox
velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/lib/python3/dist-packages/velodyne_pointcloud/cfg/TransformNodeConfig.py
velodyne_pointcloud_gencfg: /home/unita/erp_final_ws/devel/share/velodyne_pointcloud/docs/TransformNodeConfig.wikidoc
velodyne_pointcloud_gencfg: velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/build.make

.PHONY : velodyne_pointcloud_gencfg

# Rule to build all files generated by this target.
velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/build: velodyne_pointcloud_gencfg

.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/build

velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/clean:
	cd /home/unita/erp_final_ws/build/velodyne/velodyne_pointcloud && $(CMAKE_COMMAND) -P CMakeFiles/velodyne_pointcloud_gencfg.dir/cmake_clean.cmake
.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/clean

velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/depend:
	cd /home/unita/erp_final_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unita/erp_final_ws/src /home/unita/erp_final_ws/src/velodyne/velodyne_pointcloud /home/unita/erp_final_ws/build /home/unita/erp_final_ws/build/velodyne/velodyne_pointcloud /home/unita/erp_final_ws/build/velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : velodyne/velodyne_pointcloud/CMakeFiles/velodyne_pointcloud_gencfg.dir/depend

