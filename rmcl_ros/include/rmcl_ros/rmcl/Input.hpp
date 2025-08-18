#ifndef RMCL_MCL_INPUT_HPP
#define RMCL_MCL_INPUT_HPP

namespace rmcl
{

template<typename DataT>
class Input
{
public:
  void setInput(const DataT input);

protected:
  DataT input_;
};

} // namespace rmcl

#include "Input.tcc"

#endif // RMCL_MCL_INPUT_HPP
