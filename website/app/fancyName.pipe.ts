import { Pipe, PipeTransform } from '@angular/core';

/*
 * Turn True/False into On/Off for display of booleans from JSON object.
 */
@Pipe({name: 'fancyName'})
export class FancyName implements PipeTransform {
    transform(value: string): string {
        var returnValue = String(value);
        returnValue = returnValue.replace(/([A-Z]+)/g, " $1").replace(/([A-Z][a-z])/g, " $1");
        returnValue = returnValue.charAt(0).toUpperCase() + returnValue.slice(1);
        return returnValue;
    }
}
