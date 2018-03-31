import React, {Component} from 'react';

class EmergencyStopButton extends Component {

    render() {
        const btnClass = this.props.haltRobot ? 'btn btn-success btn-tight' : 'btn btn-danger btn-tight';
        const btnText = this.props.haltRobot ? 'Resume' : 'Emergency STOP';
        const signal = this.props.haltRobot ? 'unHaltRobot' : 'haltRobot';

        return (
            <span style={{float: 'right'}}>
                <button type="button" className={btnClass}
                        onClick={() => this.props.sendDataToRobot(signal)}>{btnText}
                </button>
            </span>
        );
    }
}

export default EmergencyStopButton;
