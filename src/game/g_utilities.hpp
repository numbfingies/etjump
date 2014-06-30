#ifndef g_utilities_h__
#define g_utilities_h__

#include <string>
#include <vector>
#include "g_local.hpp"

typedef float vec_t;
typedef vec_t vec2_t[2];
typedef vec_t vec3_t[3];
typedef vec_t vec4_t[4];
typedef vec_t vec5_t[5];

const std::string NEWLINE = "\n";

// C++ versions of the printing functions
void BPAll( const std::string& msg );
void BPTo( gentity_t *target, const std::string& msg );
void BeginBufferPrint();
void BufferPrint( gentity_t *ent, const std::string& msg );
void FinishBufferPrint(gentity_t *ent, bool insertNewLine=false);
void CPAll( const std::string& msg );
void CPMAll( const std::string& msg );
void CPMTo( gentity_t *target, const std::string& msg );
void CPTo( gentity_t *target, const std::string& msg );
void ChatPrintAll( const std::string& msg );
void ChatPrintTo( gentity_t *target, const std::string& msg );
void ConsolePrintAll( const std::string& msg );
void ConsolePrintTo( gentity_t *target, const std::string& msg );

// Argument handling
typedef const std::vector<std::string> *Arguments;
typedef std::vector<std::string>::const_iterator ConstArgIter;
typedef std::vector<std::string>::iterator ArgIter;
Arguments GetArgs();
Arguments GetSayArgs( int start = 0 );
// returns an empty string if not so many args
std::string SayArgv(int arg);

// Conversions
bool ToInt(const std::string& toConvert, int& value);
std::string ToString( int value );

std::string ToString( vec3_t toConvert );
std::string ToString( vec_t x, vec_t y, vec_t z );

gentity_t *PlayerGentityFromString(char *name, char *err, int size);
gentity_t *PlayerGentityFromString(const std::string& name, 
                                   std::string& err);
// Removes duplicate chars
void RemoveDuplicates(std::string& out);
void CharPtrToString( const char* p, std::string& s );

std::string GetPath( const std::string& file );
bool MapExists( const std::string& map );

std::string ValueForKey( gentity_t *ent, const std::string& key );
std::string TimeStampToString( int timeStamp );
std::string TimeStampDifferenceToString( int diff );

bool ValidGuid(std::string guid);
#endif // g_utilities_h__