# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/usr/include".split(';') if "${prefix}/include;/opt/ros/melodic/share/orocos_kdl/cmake/../../../include;/usr/include/eigen3;/usr/include" != "" else []
PROJECT_CATKIN_DEPENDS = "rosconsole;urdf".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lkdl_parser;/opt/ros/melodic/lib/liborocos-kdl.so.1.4.0;/usr/lib/x86_64-linux-gnu/libtinyxml.so;/usr/lib/x86_64-linux-gnu/libtinyxml2.so".split(';') if "-lkdl_parser;/opt/ros/melodic/lib/liborocos-kdl.so.1.4.0;/usr/lib/x86_64-linux-gnu/libtinyxml.so;/usr/lib/x86_64-linux-gnu/libtinyxml2.so" != "" else []
PROJECT_NAME = "kdl_parser"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "1.13.2"
