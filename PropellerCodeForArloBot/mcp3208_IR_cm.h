#ifndef MCP_3208_IR_CM_H
#define MCP_3208_IR_CM_H

int mcp3208_IR_cm(int);

int mcp3208_IR_cm(int channel) {
  int mcp3208reading =
      readADC(channel, MCP3208_DINOUT_PIN, MCP3208_CLK_PIN, MCP3208_CS_PIN);
  float mcp3208volts =
      (float)mcp3208reading * MCP3208_REFERENCE_VOLTAGE / 4096.0;
  int mcp3208cm =
      27.86 *
      pow(mcp3208volts,
          -1.15); // https://www.tindie.com/products/upgradeindustries/sharp-10-80cm-infrared-distance-sensor-gp2y0a21yk0f/
  return (mcp3208cm);
}

#endif /* MCP_3208_IR_CM_H */
