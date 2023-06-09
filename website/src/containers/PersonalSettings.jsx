import React, { Component } from 'react';
import { Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

class PersonalSettings extends Component {
  constructor(props) {
    super(props);

    this.cardTitle = 'Settings';
  }

  render() {
    const dataTable = [];
    if (this.props.personalData) {
      for (const key in this.props.personalData) {
        if (
          this.props.personalData.hasOwnProperty(key) &&
          typeof this.props.personalData[key] !== 'object' &&
          key[0] !== '_'
        ) {
          let data = this.props.personalData[key];
          if (data === true) {
            data = 'True';
          } else if (data === false) {
            data = 'False';
          }
          dataTable.push(
            <li key={key}>
              {key}: {data}
            </li>,
          );
        }
      }
    }
    return (
      <Card id="personal-settings-card" className="card-title">
        <CardHeader>
          <CardTitle>{this.cardTitle}</CardTitle>
        </CardHeader>
        <CardBody>
          {dataTable.length > 0 ? <ul>{dataTable}</ul> : <>Loading...</>}
        </CardBody>
      </Card>
    );
  }
}

export default PersonalSettings;
