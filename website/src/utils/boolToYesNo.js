function boolToYesNo(value) {
    let returnValue = String(value);
    if (value === true) {
        returnValue = 'Yes';
    } else if (value === false || value === null) {
        returnValue = 'No';
    }
    return returnValue;
}

export default boolToYesNo;
