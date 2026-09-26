module fabrik_status_codes
  ! The status codes, defined once.
  !
  ! They were duplicated between the solver and the ordering code, which made
  ! every use ambiguous the moment a file imported both. One home, one value.
  use, intrinsic :: iso_c_binding, only: c_int
  implicit none
  public

  integer(c_int), parameter :: FABRIK_OK = 0_c_int
  integer(c_int), parameter :: FABRIK_INVALID_ARGUMENT = 1_c_int
  integer(c_int), parameter :: FABRIK_UNREACHABLE = 2_c_int
  integer(c_int), parameter :: FABRIK_NOT_CONVERGED = 3_c_int
  integer(c_int), parameter :: FABRIK_DEGENERATE_CHAIN = 4_c_int
  ! Returned by order_dependencies only: the dependency graph has a cycle, so no
  ! valid order exists. The whole rig is refused rather than partially ordered.
  integer(c_int), parameter :: FABRIK_CYCLE = 6_c_int

end module fabrik_status_codes
