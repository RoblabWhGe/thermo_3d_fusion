#include "fair/core/math/CVector.h"

// Return Normal of CVector2's
fair::CVector2 fair::Normalized(const fair::CVector2 &a)
{
  fair::CVector2 ret(a);
  return ret.normalize();
}
// Return Normal of CVector3's
fair::CVector3 fair::Normalized(const fair::CVector3 &a)
{
  fair::CVector3 ret(a);
  return ret.normalize();
}
// Return Normal of CVector4's
fair::CVector4 fair::Normalized(const fair::CVector4 &a)
{
  fair::CVector4 ret(a);
  return ret.normalize();
}

// Dot product of two CVector2's
float fair::DotProduct(const fair::CVector2 &a, const fair::CVector2 &b) 
{
  return a.x*b.x + a.y*b.y;
}

// Dot product of two CVector3's
float fair::DotProduct(const fair::CVector3 &a, const fair::CVector3 &b) 
{
  return a.x*b.x + a.y*b.y + a.z*b.z;
}

// Dot product of two CVector4's
float fair::DotProduct(const fair::CVector4 &a, const fair::CVector4 &b) 
{
  return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
}

// Swap two CVector2's
void fair::SwapVec(fair::CVector2 &a, fair::CVector2 &b) 
{
  fair::CVector2 tmp(a);

  a = b;
  b = tmp;
}

// Swap two CVector3's
void fair::SwapVec(fair::CVector3 &a, fair::CVector3 &b) 
{
  fair::CVector3 tmp(a);

  a = b;
  b = tmp;
}

// Swap two CVector4's
void fair::SwapVec(fair::CVector4 &a, fair::CVector4 &b) 
{
  fair::CVector4 tmp(a);

  a = b;
  b = tmp;
}

// Cross product of two CVector3's
fair::CVector3 fair::CrossProduct(const fair::CVector3 &a, const fair::CVector3 &b) 
{
  return fair::CVector3(a.y*b.z - a.z*b.y,
      					a.z*b.x - a.x*b.z,
      					a.x*b.y - a.y*b.x);
}

// Are these two CVector2's nearly equal?
bool fair::NearlyEquals( const fair::CVector2& a, const fair::CVector2& b, float r ) 
{
  fair::CVector2 diff = a - b;  // difference

  return(fair::DotProduct(diff, diff) < r*r);  // radius
}

// Are these two CVector3's nearly equal?
bool fair::NearlyEquals( const fair::CVector3& a, const fair::CVector3& b, float r ) 
{
  CVector3 diff = a - b;  // difference

  return(fair::DotProduct(diff, diff) < r*r);  // radius
}

// Are these two CVector4's nearly equal?
bool fair::NearlyEquals( const fair::CVector4& a, const fair::CVector4& b, float r ) 
{
  fair::CVector4 diff = a - b;  // difference

  return(fair::DotProduct(diff, diff) < r*r);  // radius
}
