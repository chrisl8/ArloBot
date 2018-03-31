import React, { Component } from 'react';
import { Collapse, Card, CardBody, CardHeader, CardTitle } from 'reactstrap';

import './Navigation.css';

class Navigation extends Component {
  constructor(props) {
    super(props);
    this.state = {
      showMapPicker: false,
      newWaypointName: '',
      newMapName: '',
    };
  }

  componentDidMount() {
    this.props.handleUpdateNavigationElement(this.cardDiv);
  }

  setNewWaypointName = (event) => {
    this.props.sendDataToRobot('setWayPoint', this.state.newWaypointName);
    event.preventDefault();
  };

  handleNewWaypointNameChange = (event) => {
    this.setState({ newWaypointName: event.target.value });
  };

  saveMap = (event) => {
    this.props.sendDataToRobot('saveMap', this.state.newMapName);
    event.preventDefault();
  };

  handleNewMapNameChange = (event) => {
    this.setState({ newMapName: event.target.value });
  };

  handleLoadMapButton = () => {
    this.setState({ showMapPicker: true });
  };

  render() {
    let cardTitleText = <span>Navigation</span>;
    if (this.props.makeMapRunning) {
      cardTitleText = (
        <span>
          Navigation - <span style={{ fontWeight: 'bold' }}>Making a Map</span>
        </span>
      );
    } else if (this.props.makeMap) {
      cardTitleText = (
        <span>
          Navigation -{' '}
          <span style={{ fontWeight: 'bold' }}>Loading Gmapping . . .</span>
        </span>
      );
    } else if (this.props.mapName !== '') {
      cardTitleText = (
        <span>
          Navigation -{' '}
          <span style={{ fontWeight: 'bold' }}>
            Map Loaded:{' '}
            <span style={{ color: '#337ab7' }}> {this.props.mapName} </span>
          </span>
        </span>
      );
    }

    const mapList = this.props.mapList.map((map) => {
      let listItem = '';
      if (map !== 'Explore!') {
        listItem = (
          <li key={map}>
            <button
              type="button"
              className="btn btn-primary"
              onClick={() => this.props.sendDataToRobot('setMap', map)}
            >
              {map}
            </button>
          </li>
        );
      }
      return listItem;
    });

    const waypointList = this.props.wayPoints.map((waypoint) => (
      <li key={waypoint}>
        <button
          type="button"
          className="btn btn-primary"
          onClick={() => this.props.sendDataToRobot('gotoWayPoint', waypoint)}
        >
          {waypoint}
        </button>
      </li>
    ));

    return (
      <div
        ref={(element) => {
          this.cardDiv = element;
        }}
      >
        <Card id="status-card" className="card-title">
          <CardHeader onClick={() => this.props.toggle('navigation')}>
            <CardTitle>{cardTitleText}</CardTitle>
          </CardHeader>
          <Collapse isOpen={this.props.isOpen}>
            <CardBody>
              {this.props.mapName === '' &&
                !this.props.makeMap && (
                  <div>
                    <button
                      type="button"
                      className="btn btn-warning"
                      onClick={() => this.props.sendDataToRobot('makeMap')}
                    >
                      Make Map
                    </button>
                    &nbsp;or&nbsp;
                    <button
                      type="button"
                      className="btn btn-warning"
                      onClick={this.handleLoadMapButton}
                    >
                      Load Map
                    </button>
                  </div>
                )}

              {this.state.showMapPicker &&
                this.props.mapName === '' &&
                !this.props.autoExplore && (
                  <div>
                    <h3>Pick a map to load:</h3>
                    <ul>{mapList}</ul>
                  </div>
                )}

              {this.props.mapName !== '' &&
                this.props.wayPoints.length > 0 && (
                  <div>
                    <h3>Send Robot to a Destination:</h3>
                    <ul>{waypointList}</ul>

                    <form
                      id="saveNewWaypointForm"
                      name="saveNewWaypointForm"
                      style={{ lineHeight: '29px' }}
                      onSubmit={this.setNewWaypointName}
                    >
                      <input
                        type="text"
                        id="saveNewWaypoint"
                        placeholder="Enter name for current location . . ."
                        value={this.state.newWaypointName}
                        onChange={this.handleNewWaypointNameChange}
                      />
                      <input
                        type="submit"
                        value="Save Current Location"
                        className="btn btn-primary"
                      />
                    </form>
                    <p>
                      If you create a destination waypoint called
                      &quot;initial&quot; that location will be set as the
                      current robot location whenever the map is loaded.
                    </p>
                  </div>
                )}
              {!this.props.makeMapRunning &&
                this.props.mapName === '' && (
                  <p>
                    To make a new map, click on &quot;Make Map&quot;. Then you
                    can guide the robot either by setting destinations in RVIZ,
                    using the remote control feature, or even a joystick or
                    keyboard teleop if you have those working.
                  </p>
                )}
              {this.props.makeMapRunning && (
                <div>
                  <p>
                    Guide the robot either by setting destinations in RVIZ,
                    using the remote control feature, or even a joystick or
                    keyboard teleop if you have those working.
                  </p>
                  <p>
                    When you are satisfied with your map you will need to save
                    it.
                  </p>
                  <form
                    id="saveMapForm"
                    name="saveMapForm"
                    style={{ lineHeight: '29px' }}
                    onSubmit={this.saveMap}
                  >
                    <input
                      type="text"
                      id="saveMap"
                      placeholder="Enter new map name . . ."
                      value={this.state.newMapName}
                      onChange={this.handleNewMapNameChange}
                    />
                    <input
                      type="submit"
                      value="Save Map"
                      className="btn btn-primary"
                    />
                  </form>
                </div>
              )}
              <p>
                The only way to load a map after making it, or to load a
                different map, is to Stop ROS and Start ROS again. ROS has no
                ability to switch from SLAM to AMCL nor to swap maps in AMCL.
              </p>
            </CardBody>
          </Collapse>
        </Card>
      </div>
    );
  }
}

export default Navigation;

/*
            <div *ngIf="arlobotSvc.webModel.autoExplore && arlobotSvc.webModel.ROSisRunning">
                <form id='saveMapForm' name='saveMapForm'>
                    <label for="saveMap"></label>
                    <input type="text" name="saveMap" id="saveMap" style="line-height: 29px;"
                           placeholder="Enter new map name . . ."
                           [(ngModel)]="newMapName"/>
                    <button type="button"
                            class="btn btn-primary" style="float: left;"
                            (click)="sendDataToArlobot('saveMap', newMapName)">Save Map
                    </button>
                </form>
            </div>

 */
