#pragma once
// (C) Copyright 2021 by Autodesk, Inc.

#include <Base/Security/Mandatory.hh>

// Include <filesystem> but silence resulting C4995 warnings
INSECURE_INCLUDE_SECTION_BEGIN
#include <filesystem>
INSECURE_INCLUDE_SECTION_END

namespace Base
{
namespace filesystem = std::filesystem;
typedef std::error_code error_code;
} // namespace Base
