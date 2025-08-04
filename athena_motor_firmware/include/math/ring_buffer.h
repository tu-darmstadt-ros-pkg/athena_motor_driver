#pragma once

#include <array>

template<typename T, size_t Size>
class RingBuffer
{
private:
  std::array<T, Size> buffer;
  size_t head = 0;
  size_t tail = 0;
  bool full = false;

public:
  void push( const T &value )
  {
    buffer[head] = value;
    if ( full ) {
      tail = ( tail + 1 ) % Size;
    }
    head = ( head + 1 ) % Size;
    full = ( head == tail );
  }

  const T &front() const { return buffer[tail]; }

  const T &operator[]( size_t index ) const
  {
    index = tail + index;
    if ( index >= Size ) {
      index -= Size;
    }
    return buffer[index];
  }

  bool empty() const { return ( !full && ( head == tail ) ); }

  size_t size() const
  {
    if ( full ) {
      return Size;
    }
    return ( head >= tail ) ? ( head - tail ) : ( Size + head - tail );
  }

  void clear()
  {
    full = false;
    head = 0;
    tail = 0;
  }
};