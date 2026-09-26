module fabrik_pipeline_c_api
  ! Flat C bindings for the chain pipeline.
  !
  ! The interface follows one rule, so no caller has to remember which function
  ! is the odd one out:
  !
  !   every routine returns an int32_t status, and every output is written
  !   through a pointer. Optional outputs are a `type(c_ptr)` that may be NULL.
  !   All buffers are flat, contiguous and row-major, in the same layout as
  !   fabrik_solve_f32.
  !
  ! This module is the only impure part of the pipeline, and that is deliberate.
  ! fabrik_geom and fabrik_pipeline are entirely pure; input screening lives here
  ! because the C ABI is the boundary, and because the screening here uses
  ! ieee_is_finite, which is immune to the floating-point flags that would defeat
  ! the pure `all_finite` used one layer down.
  !
  ! Contract shared with fabrik_solve_f32: on a rejected call the output buffers
  ! are left untouched, so a caller can never read uninitialised memory.
  use, intrinsic :: iso_c_binding, only: c_float, c_int, c_ptr, c_associated, c_f_pointer
  use, intrinsic :: iso_fortran_env, only: real32
  use, intrinsic :: ieee_arithmetic, only: ieee_is_finite
  use fabrik_status_codes
  use fabrik_pipeline
  use fabrik_order
  implicit none
  private

contains

  ! Reject a batch of buffers in one place, so every routine below screens the
  ! same way and none of them can forget.
  pure function screens_finite(values) result(ok)
    real(real32), intent(in) :: values(:)
    logical :: ok
    ok = all(ieee_is_finite(values))
  end function screens_finite

  integer(c_int) function fabrik_measure_lengths_f32(joints, joint_count, out_lengths) &
      result(status) bind(c, name="fabrik_measure_lengths_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    real(c_float), intent(out) :: out_lengths(*)
    real(c_float) :: flat(3 * joint_count)
    real(real32) :: joints_2d(3, joint_count), lengths(joint_count - 1)

    if (joint_count < 2) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    lengths = measure_lengths(int(joint_count), joints_2d)
    out_lengths(1:joint_count - 1) = real(lengths, c_float)
    status = FABRIK_OK
  end function fabrik_measure_lengths_f32

  integer(c_int) function fabrik_residual_f32(joints, joint_count, target_xyz, out_residual) &
      result(status) bind(c, name="fabrik_residual_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    real(c_float), intent(in) :: target_xyz(3)
    type(c_ptr), value, intent(in) :: out_residual
    real(c_float), pointer :: residual_ptr(:)
    real(c_float) :: flat(3 * joint_count)
    real(real32) :: joints_2d(3, joint_count)

    if (joint_count < 2 .or. .not. c_associated(out_residual)) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    call c_f_pointer(out_residual, residual_ptr, [1_c_int])
    residual_ptr(1) = real(residual_of(int(joint_count), joints_2d, &
        real(target_xyz, real32)), c_float)
    status = FABRIK_OK
  end function fabrik_residual_f32

  integer(c_int) function fabrik_apply_pole_f32(joints, joint_count, pole_xyz, out_joints) &
      result(status) bind(c, name="fabrik_apply_pole_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    real(c_float), intent(in) :: pole_xyz(3)
    real(c_float), intent(inout) :: out_joints(*)
    real(c_float) :: flat(3 * joint_count)
    real(real32) :: joints_2d(3, joint_count)

    if (joint_count < 2) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    call apply_pole(int(joint_count), joints_2d, real(pole_xyz, real32))
    out_joints(1:3 * joint_count) = reshape(joints_2d, [3 * joint_count])
    status = FABRIK_OK
  end function fabrik_apply_pole_f32

  integer(c_int) function fabrik_apply_joint_limits_f32(joints, joint_count, joint_limits, &
      limit_count, limit_passes, out_joints, out_projections, out_violations) &
      result(status) bind(c, name="fabrik_apply_joint_limits_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count, limit_count, limit_passes
    real(c_float), intent(in) :: joint_limits(*)
    real(c_float), intent(inout) :: out_joints(*)
    type(c_ptr), value, intent(in) :: out_projections, out_violations
    real(c_float), pointer :: projections_ptr(:), violations_ptr(:)
    real(c_float) :: flat(3 * joint_count)
    real(real32) :: joints_2d(3, joint_count), limits_2d(2, max(1, limit_count))
    integer(c_int) :: projections, violations

    if (joint_count < 2 .or. limit_count < 0) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (limit_count > 0) then
      if (.not. screens_finite(joint_limits(1:2 * limit_count))) then
        status = FABRIK_INVALID_ARGUMENT
        return
      end if
      limits_2d = reshape(joint_limits(1:2 * limit_count), [2, limit_count])
    else
      limits_2d = 0.0_real32
    end if
    call apply_joint_limits(int(joint_count), joints_2d, limits_2d, &
        int(limit_passes), projections, violations)
    out_joints(1:3 * joint_count) = reshape(joints_2d, [3 * joint_count])
    if (c_associated(out_projections)) then
      call c_f_pointer(out_projections, projections_ptr, [1_c_int])
      projections_ptr(1) = int(projections, c_int)
    end if
    if (c_associated(out_violations)) then
      call c_f_pointer(out_violations, violations_ptr, [1_c_int])
      violations_ptr(1) = int(violations, c_int)
    end if
    status = FABRIK_OK
  end function fabrik_apply_joint_limits_f32

  integer(c_int) function fabrik_derive_rotations_f32(joints, joint_count, out_quaternions) &
      result(status) bind(c, name="fabrik_derive_rotations_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    real(c_float), intent(inout) :: out_quaternions(*)
    real(c_float) :: flat(3 * joint_count)
    real(real32) :: joints_2d(3, joint_count), quaternions(4, joint_count)

    if (joint_count < 2) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    quaternions = 0.0_real32
    call derive_rotations(int(joint_count), joints_2d, quaternions)
    out_quaternions(1:4 * joint_count) = reshape(quaternions, [4 * joint_count])
    status = FABRIK_OK
  end function fabrik_derive_rotations_f32

  integer(c_int) function fabrik_smooth_rotations_f32(joints, joint_count, lengths, &
      previous_quaternions, quaternions, smoothing, out_joints, out_quaternions) &
      result(status) bind(c, name="fabrik_smooth_rotations_f32")
    real(c_float), intent(in) :: joints(*)
    integer(c_int), value :: joint_count
    real(c_float), intent(in) :: lengths(*)
    real(c_float), intent(in) :: previous_quaternions(*)
    real(c_float), intent(in) :: quaternions(*)
    real(c_float), value, intent(in) :: smoothing
    real(c_float), intent(inout) :: out_joints(*)
    real(c_float), intent(inout) :: out_quaternions(*)
    real(c_float) :: flat(3 * joint_count), previous_flat(4 * joint_count)
    real(c_float) :: quaternions_flat(4 * joint_count)
    real(real32) :: joints_2d(3, joint_count), lengths_1d(joint_count - 1)
    real(real32) :: previous_2d(4, joint_count), quaternions_2d(4, joint_count)
    real(real32) :: out_joints_2d(3, joint_count), out_quaternions_2d(4, joint_count)

    if (joint_count < 2) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    flat = joints(1:3 * joint_count)
    joints_2d = reshape(flat, [3, joint_count])
    previous_flat = previous_quaternions(1:4 * joint_count)
    previous_2d = reshape(previous_flat, [4, joint_count])
    quaternions_flat = quaternions(1:4 * joint_count)
    quaternions_2d = reshape(quaternions_flat, [4, joint_count])
    if (.not. screens_finite(reshape(joints_2d, [3 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (.not. screens_finite(reshape(previous_2d, [4 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (.not. screens_finite(reshape(quaternions_2d, [4 * joint_count]))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (.not. screens_finite(lengths(1:joint_count - 1))) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    lengths_1d = real(lengths(1:joint_count - 1), real32)
    call smooth_rotations(int(joint_count), joints_2d, lengths_1d, previous_2d, &
        quaternions_2d, real(smoothing, real32), out_joints_2d, out_quaternions_2d)
    out_joints(1:3 * joint_count) = reshape(out_joints_2d, [3 * joint_count])
    out_quaternions(1:4 * joint_count) = reshape(out_quaternions_2d, [4 * joint_count])
    status = FABRIK_OK
  end function fabrik_smooth_rotations_f32

  integer(c_int) function fabrik_blend_influence_f32(current_xyz, goal_xyz, influence, out_xyz) &
      result(status) bind(c, name="fabrik_blend_influence_f32")
    real(c_float), intent(in) :: current_xyz(3), goal_xyz(3)
    real(c_float), value, intent(in) :: influence
    real(c_float), intent(out) :: out_xyz(3)
    real(real32) :: blended(3)

    if (.not. screens_finite(current_xyz) .or. .not. screens_finite(goal_xyz) .or. &
        .not. screens_finite([influence])) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    blended = blend_influence(real(current_xyz, real32), real(goal_xyz, real32), &
        real(influence, real32))
    out_xyz = real(blended, c_float)
    status = FABRIK_OK
  end function fabrik_blend_influence_f32

  integer(c_int) function fabrik_order_dependencies_f32(node_count, edges, edge_count, out_order) &
      result(status) bind(c, name="fabrik_order_dependencies_f32")
    integer(c_int), value :: node_count, edge_count
    type(c_ptr), value, intent(in) :: edges
    type(c_ptr), value, intent(in) :: out_order
    integer(c_int), pointer :: order_ptr(:), edges_ptr(:)
    integer(c_int) :: order(max(1, node_count))
    integer(c_int) :: edge_matrix(2, max(1, edge_count))

    if (node_count < 0 .or. edge_count < 0) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (edge_count > 0 .and. .not. c_associated(edges)) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (edge_count > 0) then
      call c_f_pointer(edges, edges_ptr, [2_c_int * int(edge_count, c_int)])
      edge_matrix = reshape(edges_ptr, [2, int(edge_count, c_int)])
    end if
    status = order_dependencies(int(node_count, c_int), edge_matrix, int(edge_count, c_int), order)
    ! On a cycle order_dependencies refuses, and a truncated order would solve
    ! half a rig. Nothing reaches the caller in that case.
    if (status == FABRIK_OK .and. node_count > 0) then
      if (.not. c_associated(out_order)) then
        status = FABRIK_INVALID_ARGUMENT
        return
      end if
      call c_f_pointer(out_order, order_ptr, [int(node_count, c_int)])
      order_ptr = order(1:node_count)
    end if
  end function fabrik_order_dependencies_f32

end module fabrik_pipeline_c_api
