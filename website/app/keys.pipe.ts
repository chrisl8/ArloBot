// Itterate over object, as ngFor only works on arrays.
// http://stackoverflow.com/a/35647396/4982408
import { Pipe, PipeTransform } from '@angular/core';

@Pipe({name: 'keys'})
export class KeysPipe implements PipeTransform {
    transform(value, args:string[]) : any {
        let keys = [];
        for (let key in value) {
            keys.push({key: key, value: value[key]});
        }
        return keys;
    }
}