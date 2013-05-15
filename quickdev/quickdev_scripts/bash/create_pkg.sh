#!/bin/bash

###########################################################################
#  scripts/create_pkg.sh
#  --------------------
#
#  Copyright (c) 2011, Edward T. Kaszubski ( ekaszubski@gmail.com )
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are
#  met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following disclaimer
#    in the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of usc-ros-pkg nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
###########################################################################

usage()
{
        echo ""
        echo "Usage: create_pkg package [-u user1 -u user2 ...] [-d dependency1 -d dependency2 ...] [-p project] [-i include_dir] [-n Node1Class -n Node2Class ...] [-l Nodelet1Class -l Nodelet2Class ...] [-s Source1Class -s Source2Class ...]"
        echo ""
}

addDep()
{
	if [ "$deps" == "" ]; then
		deps=$1
	else
		deps="$deps $1"
	fi
}

addSource()
{
	if [ "$sources" == "" ]; then
		sources=$1
	else
		sources="$sources $1"
	fi
}

addNode()
{
	if [ "$nodes" == "" ]; then
		nodes=$1
	else
		nodes="$nodes $1"
	fi
}

addNodelet()
{
	if [ "$nodelets" == "" ]; then
		nodelets=$1
	else
		nodelets="$nodelets $1"
	fi
}

if [ $# -le 0 ]; then
        usage
        exit
fi

manifest_file="manifest.xml"
makefile_file="Makefile"
cmakelists_file="CMakeLists.txt"
doxygen_file="mainpage.dox"
nodelet_plugins_file="nodelets/nodelet_plugins.xml"
add_license_tool="quickdevadd_license"

default_user=$ROS_USER;
if [ "$default_user" == "" ]; then default_user=`whoami`; fi

package=$1; shift

while [ "$1" != "" ]; do
	case $1 in
		-u )    	shift
					if [ "$users" == "" ]; then
						users="$1"
						users_cmd="-u $1"
					else
						users="$users $1"
						users_cmd="$users -u $1"
					fi
					shift
					;;
		-d )     	shift
					addDep "$1"
					shift
					;;
		-s )    	shift
					addSource "$1"
					shift
					;;
		-n )    	shift
					addNode "$1"
					shift
					;;
		-l )    	shift
					addNodelet "$1"
					shift
					;;
		-nl )       shift
					addNode "$1"
					addNodelet "$1"
					shift
					;;
		-i )		shift
					include_dir=$1
					shift
					;;
		--help )    usage
					exit
					;;
		-p )        shift
					project=$1
					shift
					;;
	esac
done

if [ "$package" == "" ]; then usage; exit; fi
if [ "$project" == "" ]; then project=$package; fi
if [ "$users" == "" ]; then users=$default_user; fi
if [ "$include_dir" == "" ]; then include_dir=$package; fi

echo ""
echo "Creating package $package with authors { $users }, dependencies { $deps }, sources { $sources }, nodes { $nodes }, and nodelets { $nodelets }."
echo ""

mkdir $package
cd $package

echo "<package>
  <description brief=\"$package\">

     $package

  </description>
  <author>$users</author>" >> $manifest_file

#for user in $users; do
#echo "
#  <author>$user</author>" >> $manifest_file;
#done

echo "  <license>BSD</license>
  <review status=\"unreviewed\" notes=\"\"/>
  <url>http://ros.org/wiki/$package</url>" >> $manifest_file

if [ "$nodelets" != "" ]; then deps="nodelet $deps"; fi
if [ "$sources" != "" ] || [ "$nodes" != "" ] || [ "$nodelets" != "" ]; then  deps="quickdev_cpp $deps"; fi

for dep in $deps; do
  echo "  <depend package=\"$dep\"/>" >> $manifest_file
done

if [ "$sources" != "" ] || [ "$nodelets" != "" ];
then
  echo "  <export>" >> $manifest_file

  if [ "$sources" != "" ]; then echo "    <cpp cflags=\"-I\${prefix}/include\" lflags=\"-Wl,-rpath,\${prefix}/lib -L\${prefix}/lib -l$package\"/>" >> $manifest_file; fi
  if [ "$nodelets" != "" ]; then echo "    <nodelet plugin=\"\${prefix}/$nodelet_plugins_file\"/>" >> $manifest_file; fi

  echo "  </export>" >> $manifest_file;
fi

echo "</package>" >> $manifest_file

#if [ "$deps[0]" == "quickdev_cpp" ]; then
	echo 'include $(shell rospack find quickdev_cpp)/cmake.mk' >> $makefile_file
#else
#	echo 'include $(shell rospack find mk)/cmake.mk' >> $makefile_file
#fi

echo 'cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()' >> $cmakelists_file

if [ "$sources" != "" ]; then echo 'rosbuild_include( quickdev_cpp add_library_auto )' >> $cmakelists_file; fi

echo '

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# uncomment if you have defined messages
#rosbuild_genmsg()
# uncomment if you have defined services
#rosbuild_gensrv()
# uncomment if you have defined dynamic reconfigure files
# Note: requires "rosbuild_import( quickdev_cpp dynamic_reconfigure )"
#quickdev_gencfg()
' >> $cmakelists_file

if [ "$sources" != "" ]; then echo 'add_subdirectory( src )' >> $cmakelists_file; fi
if [ "$nodes" != "" ]; then echo 'add_subdirectory( nodes )' >> $cmakelists_file; fi
if [ "$nodelets" != "" ]; then echo 'add_subdirectory( nodelets )' >> $cmakelists_file; fi

echo "/**
\mainpage
\htmlinclude manifest.html

\b $package is ...

<!--
Provide an overview of your package.
-->


\section codeapi Code API

<!--
Provide links to specific auto-generated API documentation within your
package that is of particular interest to a reader. Doxygen will
document pretty much every part of your code, so do your best here to
point the reader to the actual API.

If your codebase is fairly large or has different sets of APIs, you
should use the doxygen 'group' tag to keep these APIs together. For
example, the roscpp documentation has 'libros' group.
-->


*/" >> $doxygen_file

if [ "$sources" != "" ] || [ "$nodes" != "" ]; then mkdir -p include/$include_dir; fi

if [ "$sources" != "" ]; then

mkdir src
echo 'add_library_auto( ${PROJECT_NAME} *.cpp *.cc *.c )' >> src/$cmakelists_file

fi

if [ "$nodes" != "" ]; then

mkdir nodes
echo "# gather all sources in current dir using relative filenames
file( GLOB ALL_SOURCES RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )

foreach( source \${ALL_SOURCES} )
	# ALL_SOURCES = foo.cpp;bar.cpp
	# source = foo.cpp
	# source_name_base = foo
	# source_src = foo.cpp
	# source_name = foo
	get_filename_component( source_name_base \${source} NAME_WE )
	set( source_name \${source_name_base} )
	set( source_src \${source} )

	# rosbuild_add_executable( foo foo.cpp )
	rosbuild_add_executable( \${source_name} \${source_src} )
endforeach( source )" >> nodes/$cmakelists_file

fi

if [ "$nodelets" != "" ]; then

mkdir nodelets
echo "# gather all sources in current dir using relative filenames
file( GLOB NODELET_SOURCES RELATIVE \${CMAKE_CURRENT_SOURCE_DIR} *.cpp *.cc *.c )
rosbuild_add_library( \${PROJECT_NAME}_nodelets \${NODELET_SOURCES} )" >> nodelets/$cmakelists_file

echo "<library path=\"lib/lib$package""_nodelets\">" >> $nodelet_plugins_file

fi

# convert SomeEXPRClassName to some_expr_class_name
# convert [a][AA] -> [a]_[AA] (all); convert [Aa] -> _[Aa] (2nd occurance onward) | convert A -> a (all)
#convert_camel_case_to_snake_case="sed 's:\([a-z]\)\([A-Z][A-Z]\):\1_\2:g; s:[A-Z][a-z]:_&:2g' | tr A-Z a-z"

#convert_lower_to_upper="tr a-z A-Z"
#convert_remove_underscores="tr -d _"

#make everything upper-case and remove underscores
#convert_filename_component_to_ifdef_component="$convert_lower_to_upper | $convert_remove_underscores"

for source in $sources; do
  source_file=`echo $source | sed 's:\([a-z]\)\([A-Z]\):\1_\2:g; s:\([A-Z]\)\([A-Z][a-z]\):\1_\2:g' | tr A-Z a-z`
  source_h_relpath="include/$include_dir/$source_file.h"
  source_cpp_relpath="src/$source_file.cpp"

ifdef_string=`echo $package | tr a-z A-Z | tr -d _`"_"
ifdef_string="$ifdef_string"`echo $include_dir | tr a-z A-Z | tr -d _`"_"
ifdef_string="$ifdef_string"`echo $source_file | tr a-z A-Z | tr -d _`"_H_"

echo "#ifndef $ifdef_string
#define $ifdef_string

class $source
{
	//
};

#endif // $ifdef_string" >> $source_h_relpath
  echo "#include <$include_dir/$source_file.h>" >> $source_cpp_relpath

  license_files="$license_files $source_h_relpath $source_cpp_relpath"
done

for node in $nodes; do
  node_file=`echo "$node" | sed 's:\([a-z]\)\([A-Z]\):\1_\2:g; s:\([A-Z]\)\([A-Z][a-z]\):\1_\2:g' | tr A-Z a-z`
  node_h_relpath="include/$include_dir/$node_file.h"
  node_cpp_relpath="nodes/$node_file.cpp"

ifdef_string=`echo $package | tr a-z A-Z | tr -d _`"_"
ifdef_string="$ifdef_string"`echo $include_dir | tr a-z A-Z | tr -d _`"_"
ifdef_string="$ifdef_string"`echo $node_file | tr a-z A-Z | tr -d _`"_H_"

  echo "#ifndef $ifdef_string
#define $ifdef_string

#include <quickdev/node.h>

QUICKDEV_DECLARE_NODE( $node )

QUICKDEV_DECLARE_NODE_CLASS( $node )
{
	QUICKDEV_DECLARE_NODE_CONSTRUCTOR( $node )
	{
		//
	}

	QUICKDEV_SPIN_FIRST
	{
		initAll();
	}

	QUICKDEV_SPIN_ONCE
	{
		//
	}
};

#endif // $ifdef_string" >> $node_h_relpath

  echo "#include <$include_dir/$node_file.h>
QUICKDEV_INST_NODE( $node""Node, \"$node_file\" )" >> $node_cpp_relpath

  license_files="$license_files $node_h_relpath $node_cpp_relpath"
done

for nodelet in $nodelets; do
  nodelet_file=`echo "$nodelet" | sed 's:\([a-z]\)\([A-Z]\):\1_\2:g; s:\([A-Z]\)\([A-Z][a-z]\):\1_\2:g' | tr A-Z a-z`
  nodelet_cpp_relpath="nodelets/$nodelet_file.cpp"

  echo "#include <quickdev/nodelet.h>
#include <$include_dir/$nodelet_file.h>

QUICKDEV_DECLARE_NODELET( $package, $nodelet )

QUICKDEV_INST_NODELET( $package, $nodelet, $nodelet_file )" >> $nodelet_cpp_relpath

echo "  <class name=\"$package/$nodelet_file\" type=\"$package::$nodelet""Nodelet\" base_class_type=\"nodelet::Nodelet\">
    <description>
      todo: fill this in
    </description>
  </class>" >> $nodelet_plugins_file

  license_files="$license_files $nodelet_cpp_relpath"
done

if [ "$nodelets" != "" ]; then echo "</library>" >> $nodelet_plugins_file; fi

if [ "$license_files" != "" ]; then

echo ""
echo "Using `which $add_license_tool`"
echo ""

$add_license_tool -p $project $users_cmd $license_files

fi

echo "done"
