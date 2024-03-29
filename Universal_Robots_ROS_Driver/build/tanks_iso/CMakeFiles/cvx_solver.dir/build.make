# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/federico/benzi_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/federico/benzi_ws/src/build

# Include any dependencies generated for this target.
include tanks_iso/CMakeFiles/cvx_solver.dir/depend.make

# Include the progress variables for this target.
include tanks_iso/CMakeFiles/cvx_solver.dir/progress.make

# Include the compile flags for this target's objects.
include tanks_iso/CMakeFiles/cvx_solver.dir/flags.make

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o: tanks_iso/CMakeFiles/cvx_solver.dir/flags.make
tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o: ../tanks_iso/lib/ldl.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cvx_solver.dir/lib/ldl.c.o   -c /home/federico/benzi_ws/src/tanks_iso/lib/ldl.c

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cvx_solver.dir/lib/ldl.c.i"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/federico/benzi_ws/src/tanks_iso/lib/ldl.c > CMakeFiles/cvx_solver.dir/lib/ldl.c.i

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cvx_solver.dir/lib/ldl.c.s"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/federico/benzi_ws/src/tanks_iso/lib/ldl.c -o CMakeFiles/cvx_solver.dir/lib/ldl.c.s

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.requires:

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.requires

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.provides: tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.requires
	$(MAKE) -f tanks_iso/CMakeFiles/cvx_solver.dir/build.make tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.provides.build
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.provides

tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.provides.build: tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o


tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o: tanks_iso/CMakeFiles/cvx_solver.dir/flags.make
tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o: ../tanks_iso/lib/solver.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cvx_solver.dir/lib/solver.c.o   -c /home/federico/benzi_ws/src/tanks_iso/lib/solver.c

tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cvx_solver.dir/lib/solver.c.i"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/federico/benzi_ws/src/tanks_iso/lib/solver.c > CMakeFiles/cvx_solver.dir/lib/solver.c.i

tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cvx_solver.dir/lib/solver.c.s"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/federico/benzi_ws/src/tanks_iso/lib/solver.c -o CMakeFiles/cvx_solver.dir/lib/solver.c.s

tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.requires:

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.requires

tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.provides: tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.requires
	$(MAKE) -f tanks_iso/CMakeFiles/cvx_solver.dir/build.make tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.provides.build
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.provides

tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.provides.build: tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o


tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o: tanks_iso/CMakeFiles/cvx_solver.dir/flags.make
tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o: ../tanks_iso/lib/matrix_support.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o   -c /home/federico/benzi_ws/src/tanks_iso/lib/matrix_support.c

tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cvx_solver.dir/lib/matrix_support.c.i"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/federico/benzi_ws/src/tanks_iso/lib/matrix_support.c > CMakeFiles/cvx_solver.dir/lib/matrix_support.c.i

tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cvx_solver.dir/lib/matrix_support.c.s"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/federico/benzi_ws/src/tanks_iso/lib/matrix_support.c -o CMakeFiles/cvx_solver.dir/lib/matrix_support.c.s

tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.requires:

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.requires

tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.provides: tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.requires
	$(MAKE) -f tanks_iso/CMakeFiles/cvx_solver.dir/build.make tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.provides.build
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.provides

tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.provides.build: tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o


tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o: tanks_iso/CMakeFiles/cvx_solver.dir/flags.make
tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o: ../tanks_iso/lib/util.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cvx_solver.dir/lib/util.c.o   -c /home/federico/benzi_ws/src/tanks_iso/lib/util.c

tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cvx_solver.dir/lib/util.c.i"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/federico/benzi_ws/src/tanks_iso/lib/util.c > CMakeFiles/cvx_solver.dir/lib/util.c.i

tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cvx_solver.dir/lib/util.c.s"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/federico/benzi_ws/src/tanks_iso/lib/util.c -o CMakeFiles/cvx_solver.dir/lib/util.c.s

tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.requires:

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.requires

tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.provides: tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.requires
	$(MAKE) -f tanks_iso/CMakeFiles/cvx_solver.dir/build.make tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.provides.build
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.provides

tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.provides.build: tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o


tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o: tanks_iso/CMakeFiles/cvx_solver.dir/flags.make
tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o: ../tanks_iso/lib/variables_definition.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o   -c /home/federico/benzi_ws/src/tanks_iso/lib/variables_definition.c

tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/cvx_solver.dir/lib/variables_definition.c.i"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/federico/benzi_ws/src/tanks_iso/lib/variables_definition.c > CMakeFiles/cvx_solver.dir/lib/variables_definition.c.i

tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/cvx_solver.dir/lib/variables_definition.c.s"
	cd /home/federico/benzi_ws/src/build/tanks_iso && /usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/federico/benzi_ws/src/tanks_iso/lib/variables_definition.c -o CMakeFiles/cvx_solver.dir/lib/variables_definition.c.s

tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.requires:

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.requires

tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.provides: tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.requires
	$(MAKE) -f tanks_iso/CMakeFiles/cvx_solver.dir/build.make tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.provides.build
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.provides

tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.provides.build: tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o


# Object files for target cvx_solver
cvx_solver_OBJECTS = \
"CMakeFiles/cvx_solver.dir/lib/ldl.c.o" \
"CMakeFiles/cvx_solver.dir/lib/solver.c.o" \
"CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o" \
"CMakeFiles/cvx_solver.dir/lib/util.c.o" \
"CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o"

# External object files for target cvx_solver
cvx_solver_EXTERNAL_OBJECTS =

devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/build.make
devel/lib/libcvx_solver.so: tanks_iso/CMakeFiles/cvx_solver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/federico/benzi_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C shared library ../devel/lib/libcvx_solver.so"
	cd /home/federico/benzi_ws/src/build/tanks_iso && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cvx_solver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tanks_iso/CMakeFiles/cvx_solver.dir/build: devel/lib/libcvx_solver.so

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/build

tanks_iso/CMakeFiles/cvx_solver.dir/requires: tanks_iso/CMakeFiles/cvx_solver.dir/lib/ldl.c.o.requires
tanks_iso/CMakeFiles/cvx_solver.dir/requires: tanks_iso/CMakeFiles/cvx_solver.dir/lib/solver.c.o.requires
tanks_iso/CMakeFiles/cvx_solver.dir/requires: tanks_iso/CMakeFiles/cvx_solver.dir/lib/matrix_support.c.o.requires
tanks_iso/CMakeFiles/cvx_solver.dir/requires: tanks_iso/CMakeFiles/cvx_solver.dir/lib/util.c.o.requires
tanks_iso/CMakeFiles/cvx_solver.dir/requires: tanks_iso/CMakeFiles/cvx_solver.dir/lib/variables_definition.c.o.requires

.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/requires

tanks_iso/CMakeFiles/cvx_solver.dir/clean:
	cd /home/federico/benzi_ws/src/build/tanks_iso && $(CMAKE_COMMAND) -P CMakeFiles/cvx_solver.dir/cmake_clean.cmake
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/clean

tanks_iso/CMakeFiles/cvx_solver.dir/depend:
	cd /home/federico/benzi_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/federico/benzi_ws/src /home/federico/benzi_ws/src/tanks_iso /home/federico/benzi_ws/src/build /home/federico/benzi_ws/src/build/tanks_iso /home/federico/benzi_ws/src/build/tanks_iso/CMakeFiles/cvx_solver.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : tanks_iso/CMakeFiles/cvx_solver.dir/depend

