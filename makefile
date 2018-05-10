# AGENTS_FILE:            File containing agent definitions
# SOURCES:                List of all XABSL source files
#
# XABSL_OUTPUT_DIR:       Directory where the intermediate code shall be generated
# XABSL_TEMP_DIR:         Directory where XABSL compiler will store temporary files
# INTERMEDIATE_CODE:      The filename of the intermediate code to be generated
# DEBUG_SYMBOLS:          The filename of the debug symbols to be generated
# XML_OUTPUT_DIR:         Directory where the xml files should be generated
# DOC_OUTPUT_DIR:         Directory for the documentation output
#
# XABSL_COMPILER_DIR:     Directory which contains the Xabsl compiler
# XABSL_COMPILER_OPTIONS: Additional optional parameters for the XABSL compiler
#
# XSLT:                   An XSLT processor that can process XInclude statements (with necessary parameters)
# DOT:                    Path of the dot tool
# DOTML_DIR:              Directory that contains the DotML Schemas and XSLT stylesheets
# XABSL_XSL_DIR:          Directory which contains the Xabsl XSLT Stylesheets

XABSL_DIR 				= ../Xabsl
XSLT = /usr/local/bin/xsltproc.exe --xinclude

XABSL_COMPILER_DIR 			= ../Xabsl/compiler
XABSL_XSL_DIR 			= ../Xabsl/xsl

XABSL_OUTPUT_DIR 			= ../intermediate_code
INTERMEDIATE_CODE 			= intermediate_2016.dat
DEBUG_SYMBOLS 			= debug-symbols.dat
DOC_OUTPUT_DIR			= ./xabsl-doc
XABSL_TEMP_DIR			= .
XML_OUTPUT_DIR 			= ./xabsl-xml
XABSL_COMPILER_OPTIONS 	=  

SOURCES 				= $(shell find . -name "*.xabsl")

AGENTS_FILE          	= ./xabsl_agents.xabsl

include $(XABSL_DIR)/XabslMakefile
