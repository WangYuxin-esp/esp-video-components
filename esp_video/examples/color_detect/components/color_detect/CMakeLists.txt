set(src_dirs        .)

set(include_dirs    .)

set(requires dl_image opencv)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
