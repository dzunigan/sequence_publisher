
#include "string.hpp"

//const static std::string PROJECT_VERSION = "";
//const static int PROJECT_VERSION_NUMBER = ;
const static std::string PROJECT_COMMIT_ID = "Unknown";
const static std::string PROJECT_COMMIT_DATE = "Unknown";

//inline std::string GetVersionInfo() { return StringPrintf("ProjectName %s", PROJECT_VERSION.c_str()); }

inline std::string GetBuildInfo() {
  return StringPrintf("Commit %s on %s", PROJECT_COMMIT_ID.c_str(),
                      PROJECT_COMMIT_DATE.c_str());
}

