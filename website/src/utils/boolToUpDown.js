function boolToUpDown(value) {
  let returnValue = String(value);
  if (value === true) {
    returnValue = 'Up';
  } else if (value === false || value === null) {
    returnValue = 'Down';
  }
  return returnValue;
}

export default boolToUpDown;
