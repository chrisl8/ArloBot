import {NgModule}      from '@angular/core';
import {BoolToYesNo} from './booleanToYesNo.pipe';
import {BoolToOnOff} from './boolToOnOff.pipe';
import {KeysPipe} from './keys.pipe';
import {FancyName} from './fancyName.pipe';

import {BrowserModule} from '@angular/platform-browser';
import {Ng2BootstrapModule} from 'ng2-bootstrap/ng2-bootstrap';
import {AppComponent}   from './app.component';
@NgModule({
    imports: [BrowserModule, Ng2BootstrapModule],
    declarations: [AppComponent, BoolToYesNo, BoolToOnOff, KeysPipe, FancyName],
    bootstrap: [AppComponent]
})
export class AppModule {
}
