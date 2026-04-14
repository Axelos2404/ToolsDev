// (C) Copyright 2019 by Autodesk, Inc.

#ifndef BASE_ERROR_HH_INCLUDED
#define BASE_ERROR_HH_INCLUDED

#include <Base/Utils/ThrowError.hh>
#include <Base/Config/BaseDefines.hh>
#include <stdexcept>

namespace Base {

class BASEDLLEXPORT Error : public std::runtime_error
{
public:
  enum Index
  {
  #define DEFINE_ERROR(CODE, MSG) CODE,
  #include <Base/Utils/BaseErrorInc.hh>
  #undef DEFINE_ERROR
  };

public:
  //! Constructor.
  Error(const Index _idx) : Error((int)_idx) {}
  Error(const Index _idx, const char* message) : Error((int)_idx, message) {}

  // ! virtual Destructor
  virtual ~Error() {}

  // ! Get the outcome error index
  int index() const { return (int)idx_; }

  //! Return the error message 
  virtual const char* message() const;

  template <typename IndexT>
  bool operator==(const IndexT _idx) const { return (int)_idx == idx_; }

protected:
  int idx_;

protected:
  //! Constructor.
  Error(const int _idx);
  Error(const int _idx, const char* message)
      : std::runtime_error(message)
      , idx_(_idx)
    {}
};

}//namespace BASE

#define BASE_THROW_ERROR(INDEX) { THROW_ERROR_MODULE(Base, INDEX); }
#define BASE_THROW_ERROR_if(COND, INDEX) { if (COND) BASE_THROW_ERROR(INDEX); }

#define BASE_THROW_ERROR_TODO(MSG) { THROW_ERROR_TODO_MODULE(Base, MSG); }
#define BASE_THROW_ERROR_TODO_if(COND, MSG) { if (COND) BASE_THROW_ERROR_TODO(MSG); }

#endif//BASE_ERROR_HH_INCLUDED


