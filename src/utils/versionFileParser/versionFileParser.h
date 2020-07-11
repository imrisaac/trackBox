//
//  versionFileParser.h
//
//  Created by Isaac Reed on 6/10/20
//

#ifndef VERSION_FILE_PARSER_H
#define VERSION_FILE_PARSER_H

#include <glib.h>
#include <glib/gstdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>

class VersionFileParser{
public:
  VersionFileParser();
  ~VersionFileParser();
  gchar* GetPackageUpdateVersionString();
  void GetPackageUpdateVersionInt(int &release, int &major, int &minor, int &patch);
private:
  gboolean GetVersionFileContents(const gchar *file);
  gchar *version_file_contents_;
  gsize version_file_length_;
  GError *error_;
  gchar *package_version_string_;
  gint package_release_;
  gint package_major_;
  gint package_minor_;
  gint package_patch_;
};

#endif // VERSION_FILE_PARSER