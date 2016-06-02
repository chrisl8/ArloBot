import { Pipe, PipeTransform } from '@angular/core';

/*
 * Turn True/False into On/Off for display of booleans from JSON object.
 */
@Pipe({name: 'boolToOnOff'})
export class BoolToOnOff implements PipeTransform {
    transform(value: boolean): string {
        var returnValue = String(value);
        if (value === true) {
            returnValue = 'On';
        } else if (value === false) {
            returnValue = 'Off';
        }
        return returnValue;
    }
}
