import { Component, For, Show, createResource } from "solid-js";


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

const URL = 'http://drone.bimbur.art/api';
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
  detection: Detection
}

const Event = (props: EventProps) => {
  const det = props.detection;
  return ( 
    <div>
      <p>Lattitude: {det.lat}</p>
      <p>Longitude: {det.lon}</p>
      <p>{det.date + ''}</p>
      <p>{det.photo}</p>
      <img src={URL + '/upload/' + det.photo} />
      <p>Red trees: {det.trees.filter(t => t.color == 'R').length}, Blue trees: {det.trees.filter(t => t.color == 'B').length}</p>
    </div>
  );
}

export const Events: Component = () => {
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
    <Show when={data.latest} fallback={<div>Loading...</div>}>
      <div>Date: {data.latest!.queryDate + ''}</div>
      <For each={data.latest!.detections}>{det => <Event detection={det} />}</For>
    </Show>
  );
}
