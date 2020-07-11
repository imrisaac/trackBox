//
//  versionFileParser.cpp
//
//  Created by Isaac Reed on 6/10/20
//

#include "versionFileParser.h"

#ifdef UNITTEST
const gchar *VERSION_FILE = "../test_files/updateinfo.txt";
#else
const gchar *VERSION_FILE = "/updateinfo.txt";
#endif

using namespace std;

VersionFileParser::VersionFileParser(){
  package_release_ = 0;
  package_major_ = 0;
  package_minor_ = 0;
  package_patch_ = 0;
  error_ = NULL;
  GetVersionFileContents(VERSION_FILE);
}

VersionFileParser::~VersionFileParser(){
  g_free(error_);
  g_free(version_file_contents_);
}

gchar* VersionFileParser::GetPackageUpdateVersionString(){
  return package_version_string_;
}

gboolean VersionFileParser::GetVersionFileContents(const gchar *file){
  gboolean ret = g_file_test(file, G_FILE_TEST_EXISTS);
  if(ret == false){
    return ret;
  }
  ret = g_file_get_contents(file, &version_file_contents_, 
    &version_file_length_, &error_);

  // split the input string on every instance of : and \n
  gchar **tokens = g_strsplit_set(version_file_contents_, ":\n", 3);
  package_version_string_ = tokens[1];
  gchar **int_tokens = g_strsplit_set(package_version_string_, ".", 4);
  
  package_release_ = g_ascii_strtoll(int_tokens[0], NULL, 0);
  package_major_ = g_ascii_strtoll(int_tokens[1], NULL, 0);
  package_minor_ = g_ascii_strtoll(int_tokens[2], NULL, 0);
  package_patch_ = g_ascii_strtoll(int_tokens[3], NULL, 0);
  return ret;
}

void VersionFileParser::GetPackageUpdateVersionInt(int &release, int &major, int &minor, int &patch){
  release = package_release_;
  major = package_major_;
  minor = package_minor_;
  patch = package_patch_;
  return;
}
