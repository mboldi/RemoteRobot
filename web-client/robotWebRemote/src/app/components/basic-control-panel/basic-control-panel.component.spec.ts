import { ComponentFixture, TestBed } from '@angular/core/testing';

import { BasicControlPanelComponent } from './basic-control-panel.component';

describe('BasicControlPanelComponent', () => {
  let component: BasicControlPanelComponent;
  let fixture: ComponentFixture<BasicControlPanelComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ BasicControlPanelComponent ]
    })
    .compileComponents();
  });

  beforeEach(() => {
    fixture = TestBed.createComponent(BasicControlPanelComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
