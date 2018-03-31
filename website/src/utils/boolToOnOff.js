function boolToOnOff(value) {
    let returnValue = String(value);
    if (value === true) {
        returnValue = 'On';
    } else if (value === false || value === null) {
        returnValue = 'Off';
    }
    return returnValue;
}

export default boolToOnOff;
