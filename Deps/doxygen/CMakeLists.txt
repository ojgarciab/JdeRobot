# Avoid refind same package twice or more
if (NOT DEFINED DOXYGEN_FOUND)
	find_package(Doxygen)
	# Show a message tip only first time
	if(DOXYGEN_FOUND)
		MESSAGE("*** Found doxygen: use \"make doc\" to generate documentation")
	else()
		MESSAGE("*** Doxygen NOT found: if you want to generate documentation you'll need to install Doxygen")
	endif()
endif()

