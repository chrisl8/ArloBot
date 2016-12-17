import { Pipe, PipeTransform } from '@angular/core';

/*
 * Turn True/False into Yes/No for display of booleans from JSON object.
 */
@Pipe({name: 'boolToYesNo'})
export class BoolToYesNo implements PipeTransform {
    transform(value: boolean): string {
        var returnValue = String(value);
        if (value === true) {
            returnValue = 'Yes';
        } else if (value === false || value === null) {
            returnValue = 'No';
        }
        return returnValue;
    }
}
