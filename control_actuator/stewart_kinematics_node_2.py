"""Compatibility entry point for the former alternate kinematics node."""

from .stewart_kinematics_node import StewartKinematicsNode, main

__all__ = ['StewartKinematicsNode', 'main']


if __name__ == '__main__':
    main()
