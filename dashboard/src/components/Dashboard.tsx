import React from 'react';
import './Dashboard.css';

import Monitor from './PageMonitor';
import Mapping from './PageMapping';

export enum DashboardPages {
  Monitor = "Monitor",
  Mapping = "Mapping"
}

interface DashboardState {
  contentPage: DashboardPages
}

export default class Dashboard extends React.Component<{}, DashboardState> {
  constructor(props: any) {
    super(props);

    // initialize state
    this.state = {
      contentPage: DashboardPages.Monitor
    };
    
    // bind functions
    this.switchPage = this.switchPage.bind(this);
  }

  switchPage(page: DashboardPages){
    this.setState({
      contentPage: page
    });
  }
  
  render() {    
    let contentComponent;

    // determine content component
    switch(this.state.contentPage)
    {
      case DashboardPages.Monitor:
        contentComponent = <Monitor/>;
        break;
      case DashboardPages.Mapping:
        contentComponent = <Mapping />;
        break;
    }

    return <div className="container">
      <div className="sidebar">
        <h1 className="logo">Dodobot</h1>
        <ul className="menuItems">
          <li className="menuItem" onClick={() => this.switchPage(DashboardPages.Monitor)}>Monitor</li>
          <li className="menuItem" onClick={() => this.switchPage(DashboardPages.Mapping)}>Mapping</li>
        </ul>
      </div>
      <div className="content">
        {contentComponent}
      </div>
    </div>
  }
}