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

  // TODO: Feedback during/after waypoint set
  // TODO: Feedback for setting Destinations, because sometimes they don't "set", and how do I now if it has arrived?

  componentDidMount() {
    this.props.handleUpdateNavigationElement(this.cardDiv);
  }

  setNewWaypointName = (event) => {
    this.props.sendDataToRobot('setWayPoint', this.state.newWaypointName);
    this.setState({ newWaypointName: '' });
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
      if (this.props.makeMap) {
        cardTitleText = (
          <span>
            Navigation -&nbsp;
            <span style={{ fontWeight: 'bold' }}>
              Making a Map with Slam Toolbox
            </span>
          </span>
        );
      }
    } else if (this.props.makeMap) {
      cardTitleText = (
        <span>
          Navigation -&nbsp;
          <span style={{ fontWeight: 'bold' }}>
            Please wait, Loading Slam Toolbox . . .
          </span>
        </span>
      );
    } else if (this.props.mapName !== '' && this.props.mapLoaded) {
      cardTitleText = (
        <span>
          Navigation -&nbsp;
          <span style={{ fontWeight: 'bold' }}>
            <span style={{ color: '#337ab7' }}> {this.props.mapName} </span>
            &nbsp;map loaded&nbsp;
            {this.props.navigationInProgress && (
              <span>
                &nbsp;-&nbsp;Going to&nbsp;
                <span style={{ color: '#337ab7' }}>
                  {this.props.wayPointName}
                </span>
                &nbsp;...
              </span>
            )}
          </span>
        </span>
      );
    } else if (this.props.mapName !== '') {
      cardTitleText = (
        <span>
          Navigation -&nbsp;
          <span style={{ fontWeight: 'bold' }}>
            Please wait, Loading Map:&nbsp;
            <span style={{ color: '#337ab7' }}> {this.props.mapName} </span>...
          </span>
        </span>
      );
    }

    const mapList = this.props.mapList.map((map) => {
      let listItem = '';
      listItem = (
        <li key={map}>
          <button
            id={`load-map-button-${map}`}
            type="button"
            className="btn btn-primary"
            onClick={() => this.props.sendDataToRobot('setMap', map)}
          >
            {map}
          </button>
        </li>
      );
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
        <Card id="navigation-card" className="card-title">
          <CardHeader onClick={() => this.props.toggle('navigation')}>
            <CardTitle>{cardTitleText}</CardTitle>
          </CardHeader>
          <Collapse id="navigation-card-body" isOpen={this.props.isOpen}>
            <CardBody>
              {this.props.mapName === '' && !this.props.makeMap && (
                <div>
                  <button
                    id="make-map-button"
                    type="button"
                    className="btn btn-warning"
                    onClick={() => this.props.sendDataToRobot('makeMap')}
                  >
                    Make Map - Slam Toolbox
                  </button>
                  {!this.state.showMapPicker && (
                    <>
                      &nbsp;or&nbsp;
                      <button
                        id="load-map-button"
                        type="button"
                        className="btn btn-warning"
                        onClick={this.handleLoadMapButton}
                      >
                        Load Map
                      </button>
                    </>
                  )}
                </div>
              )}

              {this.state.showMapPicker &&
                this.props.mapName === '' &&
                !this.props.makeMap &&
                !this.props.makeMapRunning && (
                  <div>
                    <h3>Pick a map to load:</h3>
                    <ul>{mapList}</ul>
                  </div>
                )}

              {!this.props.pluggedIn &&
                !this.props.navigationInProgress &&
                Boolean(this.props.mapName) !== '' &&
                Boolean(this.props.mapLoaded) &&
                Boolean(this.props.wayPoints.length) > 0 && (
                  <div>
                    <h3>Send Robot to a Destination:</h3>
                    <ul>{waypointList}</ul>
                    <button
                      type="button"
                      className="btn btn-primary"
                      onClick={() =>
                        this.props.sendDataToRobot('returnToMapZeroPoint')
                      }
                    >
                      Return to Map Zero Point
                    </button>
                  </div>
                )}

              {this.props.pluggedIn &&
                Boolean(this.props.mapName) !== '' &&
                Boolean(this.props.mapLoaded) &&
                Boolean(this.props.wayPoints.length) > 0 && (
                  <div>
                    <p>
                      <strong>
                        Waypoint navigation is disabled while robot is plugged
                        in.
                      </strong>
                    </p>
                  </div>
                )}

              {this.props.navigationInProgress && (
                <p>
                  <strong>
                    Navigation to{' '}
                    <span style={{ color: '#337ab7' }}>
                      {this.props.wayPointName}
                    </span>{' '}
                    in progress...
                  </strong>
                </p>
              )}

              {Boolean(this.props.lastNavigationResult) && (
                <p>
                  <strong>
                    Last Destination:&nbsp;
                    <span style={{ color: '#337ab7' }}>
                      {this.props.wayPointName}
                    </span>
                    &nbsp; Last Navigation result:&nbsp;
                    <span style={{ color: '#337ab7' }}>
                      {this.props.lastNavigationResult}
                    </span>
                  </strong>
                </p>
              )}

              {Boolean(this.props.mapName) !== '' &&
                Boolean(this.props.mapLoaded) &&
                !this.props.navigationInProgress && (
                  <div>
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
                  </div>
                )}

              {!this.props.makeMapRunning && this.props.mapName === '' && (
                <p>
                  To make a new map, click on &quot;Make Map&quot;. Then you can
                  guide the robot either by setting destinations in RVIZ, using
                  the remote control feature, or even a joystick or keyboard
                  teleop if you have those working.
                </p>
              )}
              {this.props.makeMapRunning && (
                <div>
                  <p>
                    Guide the robot either by setting destinations in RVIZ,
                    using the remote control feature, or even a joystick or
                    keyboard teleop if you have those working.
                  </p>
                  <button
                    type="button"
                    className="btn btn-primary"
                    onClick={() =>
                      this.props.sendDataToRobot('returnToMapZeroPoint')
                    }
                  >
                    Return to Map Zero Point
                  </button>
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
                  {this.state.newMapName &&
                    this.props.mapList.indexOf(this.state.newMapName) > -1 && (
                      <p>
                        <strong>
                          Map &quot;{this.state.newMapName}&quot; has been
                          saved.
                        </strong>
                      </p>
                    )}
                </div>
              )}
              <p>
                The only way to load a map after making it, or to load a
                different map, is to Stop ROS and Start ROS again. ROS cannot
                switch from SLAM mode to Localization mode nor swap maps while
                in Localization mode.
              </p>
            </CardBody>
          </Collapse>
        </Card>
      </div>
    );
  }
}

export default Navigation;
