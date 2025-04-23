// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

public class Container<T> {
  public T value;

  public Container() {}

  public Container(T initialValue) {
    value = initialValue;
  }
}
