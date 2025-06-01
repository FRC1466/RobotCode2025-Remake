// Copyright (c) 2025 FRC 1466
// http://github.com/FRC1466
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import java.util.*;
import java.util.regex.*;

public class PathTokenParser {

  public enum TokenType {
    A("A-(\\d+(?:\\.\\d+)?)"),
    S("S"),
    P("P-([A-Z])"),
    B("B"),
    C("C-(\\d{1,2})-(\\d)"),
    I("I-(\\d+(?:\\.\\d+)?)"),
    POSE("POSE-(\\d+(?:\\.\\d+)?)-(\\d+(?:\\.\\d+)?)-(\\d+(?:\\.\\d+)?)");

    final Pattern pattern;

    TokenType(String regex) {
      this.pattern = Pattern.compile(regex);
    }
  }

  public static class Token {
    public final TokenType type;
    public final String[] values;

    public Token(TokenType type, String[] values) {
      this.type = type;
      this.values = values;
    }

    @Override
    public String toString() {
      return type.name() + "(" + String.join(",", values) + ")";
    }
  }

  public static List<Token> parse(String input) {
    List<Token> tokens = new ArrayList<>();
    String[] parts = input.split("_");

    for (String part : parts) {
      boolean matched = false;
      for (TokenType type : TokenType.values()) {
        Matcher matcher = type.pattern.matcher(part);
        if (matcher.matches()) {
          String[] values = new String[matcher.groupCount()];
          for (int i = 0; i < matcher.groupCount(); i++) {
            values[i] = matcher.group(i + 1);
          }
          tokens.add(new Token(type, values));
          matched = true;
          break;
        }
      }
      if (!matched) {
        System.out.printf("Unrecognized token: %s%n", part);
      }
    }

    System.out.printf(tokens.toString());
    return tokens;
  }
}
