#ifndef APERY_COLOR_HPP
#define APERY_COLOR_HPP

#include "overloadEnumOperators.hpp"

enum Color {
	Black, White, ColorNum
};
OverloadEnumOperators(Color);

inline constexpr Color oppositeColor(const Color c) {
	return static_cast<Color>(static_cast<int>(c) ^ 1);
}

inline Color operator~(Color c) {
  return Color(c ^ White);
}

#endif // #ifndef APERY_COLOR_HPP
