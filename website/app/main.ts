import {platformBrowserDynamic} from '@angular/platform-browser-dynamic';
import {AppModule} from './app.module';

// enable production mode and thus disable debugging information
import {enableProdMode} from '@angular/core';
enableProdMode();

const platform = platformBrowserDynamic();
platform.bootstrapModule(AppModule);
