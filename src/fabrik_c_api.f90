module fabrik_c_api
  use, intrinsic :: iso_c_binding, only: c_float, c_int, c_ptr, c_associated, c_f_pointer
  use fabrik_core, only: solve_f32, FABRIK_INVALID_ARGUMENT, FABRIK_DEGENERATE_CHAIN
  implicit none

contains

  integer(c_int) function fabrik_solve_f32(joints, joint_count, lengths, target_xyz, &
      root_anchored, tolerance, max_iterations, out_joints, out_residual) result(status) bind(c, name="fabrik_solve_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    type(c_ptr), value, intent(in) :: lengths
    real(c_float), intent(in) :: target_xyz(3)
    integer(c_int), value :: root_anchored
    real(c_float), value, intent(in) :: tolerance
    integer(c_int), value :: max_iterations
    real(c_float), intent(inout) :: out_joints(*)
    type(c_ptr), value, intent(in) :: out_residual
    real(c_float), pointer :: supplied_lengths(:), residual_ptr(:)
    real(c_float) :: flat(3 * joint_count), flat_out(3 * joint_count)
    real(c_float) :: out_coords(3, joint_count)
    real(c_float) :: residual

    if (joint_count < 2 .or. joint_count > 100000) then
      status = 1_c_int
      return
    end if
    flat = joints(1:3 * joint_count)
    if (c_associated(lengths)) then
      call c_f_pointer(lengths, supplied_lengths, [int(joint_count - 1, c_int)])
      status = solve_f32(reshape(flat, [3, joint_count]), joint_count, &
        supplied_lengths, target_xyz, root_anchored, tolerance, max_iterations, &
        out_coords, residual)
    else
      status = solve_f32(reshape(flat, [3, joint_count]), joint_count, &
        target=target_xyz, root_anchored=root_anchored, tolerance=tolerance, &
        max_iterations=max_iterations, out_joints=out_coords, residual=residual)
    end if
    ! solve_f32 intentionally leaves its output untouched for rejected input and
    ! degenerate chains. Preserve that contract across the C ABI as well; copying
    ! the local, uninitialised out_coords here would expose garbage to C callers.
    if (status /= FABRIK_INVALID_ARGUMENT .and. status /= FABRIK_DEGENERATE_CHAIN) then
      flat_out = reshape(out_coords, [3 * joint_count])
      out_joints(1:3 * joint_count) = flat_out
    end if
    if (c_associated(out_residual)) then
      call c_f_pointer(out_residual, residual_ptr, [1_c_int])
      residual_ptr(1) = residual
    end if
  end function fabrik_solve_f32

end module fabrik_c_api
