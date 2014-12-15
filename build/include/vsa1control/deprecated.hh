// Copyright (C) 2008-2013 LAAS-CNRS, JRL AIST-CNRS.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef VSA1CONTROL_DEPRECATED_HH
# define VSA1CONTROL_DEPRECATED_HH

// Define a suffix which can be used to tag a type, a function or a a
// variable as deprecated (i.e. it will emit a warning when using it).
//
// Tagging a function as deprecated:
//  void foo () VSA1CONTROL_DEPRECATED;
//
// Tagging a type as deprecated:
//  class Foo {};
//  typedef Foo Bar VSA1CONTROL_DEPRECATED;
//
// Tagging a variable as deprecated:
//  int a VSA1CONTROL_DEPRECATED = 0;
//
// The use of a macro is required as this is /not/ a standardized
// feature of C++ language or preprocessor, even if most of the
// compilers support it.
# ifdef __GNUC__
#  define VSA1CONTROL_DEPRECATED __attribute__ ((deprecated))
# else
#  ifdef _MSC_VER
#   define VSA1CONTROL_DEPRECATED __declspec (deprecated)
#  else
// If the compiler is not recognized, drop the feature.
#   define VSA1CONTROL_DEPRECATED /* nothing */
#  endif // __MSVC__
# endif // __GNUC__

#endif //! VSA1CONTROL_DEPRECATED_HH
