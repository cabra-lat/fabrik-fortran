module fabrik_order
  ! Deterministic dependency ordering for a rig of chains.
  !
  ! Pure integer graph work: the caller hands over a list of (child, parent) edges
  ! and gets back an order in which every parent is solved before its children.
  ! No engine type appears here, so the ordering is testable in `fpm test` rather
  ! than only through a GDScript test.
  !
  ! The order is Kahn's algorithm with ties broken by declaration index, so the
  ! same graph always produces the same order - a rig must not shuffle its solve
  ! order between frames. A cycle is REFUSED, not partially ordered: returning a
  ! truncated order would solve half a rig and leave the rest at last frame's
  ! pose, which is far harder to debug than a hard failure.
  use, intrinsic :: iso_c_binding, only: c_int
  use fabrik_status_codes
  implicit none
  private

  public :: order_dependencies

contains

  integer(c_int) function order_dependencies(node_count, edges, edge_count, order) result(status)
    integer(c_int), intent(in), value :: node_count, edge_count
    integer(c_int), intent(in) :: edges(2, edge_count)
    integer(c_int), intent(out) :: order(node_count)
    integer(c_int) :: indegree(node_count)
    logical :: emitted(node_count), placed
    integer(c_int) :: node, child, parent, i, j, emitted_count
    integer(c_int) :: next_index, candidate

    if (node_count < 0 .or. edge_count < 0) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (node_count == 0) then
      status = FABRIK_OK
      return
    end if

    do i = 1, node_count
      indegree(i) = 0_c_int
      emitted(i) = .false.
      order(i) = -1_c_int
    end do
    do i = 1, edge_count
      child = edges(1, i)
      parent = edges(2, i)
      if (child < 0 .or. child >= node_count .or. parent < 0 .or. parent >= node_count) then
        status = FABRIK_INVALID_ARGUMENT
        return
      end if
      indegree(child + 1_c_int) = indegree(child + 1_c_int) + 1_c_int
    end do

    emitted_count = 0_c_int
    do while (emitted_count < node_count)
      ! Lowest declaration index among the ready nodes: deterministic, and it
      ! keeps an unconstrained rig in the order it was written in.
      next_index = -1_c_int
      do i = 1, node_count
        if (emitted(i)) cycle
        if (indegree(i) /= 0_c_int) cycle
        next_index = i
        exit
      end do
      if (next_index < 0) then
        ! Nothing is ready and nodes remain: the remainder is a cycle. Refuse the
        ! whole ordering rather than return a partial one.
        status = FABRIK_CYCLE
        return
      end if
      emitted(next_index) = .true.
      emitted_count = emitted_count + 1_c_int
      order(emitted_count) = next_index - 1_c_int
      do j = 1, edge_count
        child = edges(1, j)
        parent = edges(2, j)
        if (parent + 1_c_int /= next_index) cycle
        if (emitted(child + 1_c_int)) cycle
        indegree(child + 1_c_int) = indegree(child + 1_c_int) - 1_c_int
      end do
    end do

    status = FABRIK_OK
  end function order_dependencies

end module fabrik_order
