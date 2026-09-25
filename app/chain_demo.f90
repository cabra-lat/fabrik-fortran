program chain_demo
  use, intrinsic :: iso_c_binding, only: c_int
  use, intrinsic :: iso_fortran_env, only: real32
  use fabrik_core
  implicit none

  real(real32) :: joints(3, 3), lengths(2), target(3), solved(3, 3), residual
  integer(c_int) :: status

  joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                     1.0_real32, 0.0_real32, 0.0_real32, &
                     2.0_real32, 0.0_real32, 0.0_real32], [3, 3])
  lengths = [1.0_real32, 1.0_real32]
  target = [1.0_real32, 1.0_real32, 0.0_real32]
  status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
    1.0e-5_real32, 64_c_int, solved, residual)
  print '(a,i0,a,es12.4)', 'Fortran FABRIK demo status=', status, ' residual=', residual
  print '(3(1x,es12.4))', solved(:, 1), solved(:, 2), solved(:, 3)
  if (status /= FABRIK_OK) error stop 1
end program chain_demo
