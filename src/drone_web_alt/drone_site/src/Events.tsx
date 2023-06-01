import { AppBar, Avatar, Card, Chip, CircularProgress, Container, Grid, List, ListItem, ListItemAvatar, ListItemButton, ListItemIcon, ListItemText, Paper, Slide, Stack, Toolbar, Typography, useTheme } from "@suid/material";
import logo from './logo.png';
import { Component, For, Resource, Show, createResource } from "solid-js";
import createElementRef from "@suid/system/createElementRef";
import { Key } from "@solid-primitives/keyed";
import { ContainerClasses } from "@suid/material/Container";


type Detection = {
  detection_id: number,
  date: Date, // change to date??
  lat: number,
  lon: number, 
  photo: string,
  trees: Tree[],
}

type Tree = {
  tree_id: number,
  color: string,
}


// testing delays
const delay = (ms: number) => new Promise(res => setTimeout(res, ms));

const URL = 'http://drone.bimbur.art/api/';
const fetchData = async () => {
  const queryDate = new Date();
  const resp = await fetch(URL);
//  await delay(3000); // if you want to test extreme delays
  const json = await resp.json();
  const detections: Detection[] = json.map((o: any) => ({ ...o, date: new Date(o.date) }));

  detections.sort((a, b) => b.date.getTime() - a.date.getTime());
  
  return { detections: detections, queryDate: queryDate };
};


interface EventProps {
  detection: Detection,
}


const Event = (props: EventProps) => {
  const det = props.detection;
  return (<>
    <Paper sx={{ margin: 1, padding: 1, display: 'flex' }}>
      <Avatar src={URL + 'upload/' + det.photo} sx={{ width: 120, height: 120, marginRight: 2 }} />
      <Grid container spacing={2} sx={{ flexGrow: 1 }}>
        <Grid item><Chip variant="outlined" label={'Lattitude: ' + det.lat} /></Grid>
        <Grid item><Chip variant="outlined" label={'Longitude: ' + det.lon} /></Grid>
        <Grid item><Chip variant="outlined" label={det.date + ''} /></Grid>
        <Grid item><Chip variant="outlined" label={'Red: ' + det.trees.filter(t => t.color == 'R').length} /></Grid>
        <Grid item><Chip variant="outlined" label={'Blue: ' + det.trees.filter(t => t.color == 'B').length} /></Grid>
      </Grid>    
    </Paper>
  </>);
}


export const Events: Component = () => {
  const element = createElementRef();
  const [data, { mutate, refetch }] = createResource(fetchData);

  setInterval(() => {
    // why the hell does this work
    // is this undefined behavior/accidental feature?
    // but it works really well, so I won't question it
    // (the conundrum is, that after refetch(), data() is undefined, thus 'loading' should display. but it does not????? why???)
    // apparently, the idiomatic way is to use data.latest
    if (!data.loading && !data.error) {
      refetch();
    }
  }, 1000);
  
  return (
    <>
      <AppBar position="static">
      <Toolbar>
        <Avatar src={logo} sx={{ mr: 2}} />
        <Show when={data.latest}>{d =>
<Typography variant="h6" component="div" sx={{ flexGrow: 1 }}>{'Last update: ' + d().queryDate + ''}
        </Typography>}
        </Show>
      </Toolbar>
    </AppBar>
    <Show when={data.latest} fallback={<CircularProgress />}>
      <Container maxWidth="lg" ref={element}>
        <Stack>
          <Key each={data.latest!.detections} by={item => item.detection_id}>{det => 
            <Slide direction="left" in={true} container={element.ref}>
              <Event detection={det()} />
            </Slide>
          }</Key>
        </Stack>
      </Container>
    </Show>
  </>
  );
}
