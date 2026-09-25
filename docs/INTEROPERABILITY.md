# Fortran/C interoperability notes

The public boundary is deliberately small and flat. The implementation was
checked against the standard `ISO_C_BINDING` guidance in Paul Norvig's
[Fortran/C/C++ interoperability guide](https://www.paulnorvig.com/guides/interoperability-of-fortran-with-cc.html)
and the older XL Fortran mixed-language guide supplied as a reference:
<https://www.cenapad.unicamp.br/parque/manuais/Xlf/UG77.HTM>.

Key rules used here:

- `bind(C)` procedure names are stable C symbols.
- Scalar arguments crossing the boundary use `iso_c_binding` kinds and `VALUE`.
- Arrays are passed as flat contiguous C buffers and copied into local Fortran
  arrays before solving. This avoids exposing column-major layout or descriptors.
- No Fortran derived type, allocatable, character, or compiler runtime object is
  present in the C ABI.
- The C++/Godot adapter links the Fortran runtime explicitly and tests the same
  C ABI independently of Godot.

The adapter is built with CMake and consumes an installed core prefix; it does
not compile hidden Fortran objects through ad-hoc shell commands.
